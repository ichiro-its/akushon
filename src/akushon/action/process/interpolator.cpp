// Copyright (c) 2021-2023 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <string>
#include <vector>

#include "akushon/action/process/interpolator.hpp"
#include "akushon/action/model/action.hpp"
#include "akushon/action/process/joint_process.hpp"
#include "tachimawari/joint/model/joint.hpp"

namespace akushon
{

Interpolator::Interpolator(const std::vector<Action> & actions, const Pose & initial_pose)
: actions(actions), joint_processes({}), current_pose_index(0),
  pause_time(0), init_pause(false), start_stop_time(0), init_state(true),
  current_action_index(0)
{
  for (const auto & joint : initial_pose.get_joints()) {
    auto joint_process = JointProcess(joint.get_id(), joint.get_position());
    joint_processes.insert({joint.get_id(), joint_process});
  }

  if (actions.size() != 0) {
    state = START_DELAY;
  } else {
    state = END;
  }
}

void Interpolator::process(double time)
{
  switch (state) {
    case START_DELAY:
      {
        if (init_state) {
          init_state = false;
          start_stop_time = time;
        }

        if ((time - start_stop_time) > (get_current_action().get_start_delay())) {
          change_state(PLAYING);
        }

        break;
      }

    case PLAYING:
      {
        if (check_for_next()) {
          if (init_pause) {
            init_pause = false;
            pause_time = time;
          }

          if (current_pose_index == get_current_action().get_pose_count()) {
            change_state(STOP_DELAY);
            init_pause = true;
          } else if ((time - pause_time) > (get_current_pose().get_pause())) {
            next_pose(time);
            init_pause = true;
          }
        }

        break;
      }

    case STOP_DELAY:
      {
        if (init_state) {
          init_state = false;
          start_stop_time = time;
        }

        if ((time - start_stop_time) > (get_current_action().get_stop_delay())) {
          ++current_action_index;

          if (current_action_index == actions.size()) {
            change_state(END);
          } else {
            current_pose_index = 0;
            change_state(START_DELAY);
          }
        }

        break;
      }
  }

  for (const auto & [id, joint] : joint_processes) {
    if (get_current_action().time_based)
      joint_processes.at(id).interpolate_time(time);
    else
      joint_processes.at(id).interpolate();
  }
}

bool Interpolator::is_finished() const
{
  return state == END;
}

void Interpolator::next_pose(double time)
{
  auto current_pose = get_current_pose();
  for (const auto & joint : current_pose.get_joints()) {
    if (joint_processes.find(joint.get_id()) != joint_processes.end()) {
      if(get_current_action().time_based)
        joint_processes.at(joint.get_id()).set_target_position_time(joint.get_position(), time, current_pose.action_time);
      else
        joint_processes.at(joint.get_id()).set_target_position(joint.get_position(), current_pose.get_speed());
    }
  }
  ++current_pose_index;
}

bool Interpolator::check_for_next()
{
  int joint_number = joint_processes.size();
  for (const auto & [id, joint] : joint_processes) {
    if (joint.is_finished()) {
      --joint_number;
    }
  }

  return joint_number <= 0;
}

const Action & Interpolator::get_current_action() const
{
  return actions[current_action_index];
}

const Pose & Interpolator::get_current_pose() const
{
  return get_current_action().get_pose(current_pose_index);
}

void Interpolator::change_state(int state)
{
  this->state = state;
  init_state = true;
}

std::vector<tachimawari::joint::Joint> Interpolator::get_joints() const
{
  std::vector<tachimawari::joint::Joint> joints;

  for (const auto & [id, joint] : joint_processes) {
    joints.push_back(joint);
  }

  return joints;
}

}  // namespace akushon
