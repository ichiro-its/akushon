// Copyright (c) 2021 Ichiro ITS
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
  pause_time(0), init_pause(false), start_stop_time(0), state(START_DELAY),
  init_state(true), current_action_index(0)
{
  for (const auto & joint : initial_pose.get_joints()) {
    joint_processes.insert(
      {joint.get_id(),
        JointProcess(joint.get_id(), joint.get_position())});
  }
}

void Interpolator::process(int time)
{
  switch (state) {
    case START_DELAY:
      {
        if (init_state) {
          init_state = false;
          start_stop_time = time;
        }

        if ((time - start_stop_time) > (actions[current_action_index].get_start_delay() * 1000)) {
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

          if (current_pose_index == actions[current_action_index].get_pose_count()) {
            change_state(STOP_DELAY);
            init_pause = true;
          } else if ((time - pause_time) >  // NOLINT
            (actions[current_action_index].get_pose(current_pose_index).get_pause() * 1000))
          {
            next_pose();
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

        if ((time - start_stop_time) > (actions[current_action_index].get_stop_delay() * 1000)) {
          current_action_index++;

          if (current_action_index == actions.size()) {
            change_state(END);
          } else {
            change_state(START_DELAY);
          }
        }

        break;
      }
  }

  for (auto [id, joint] : joint_processes) {
    joint_processes.at(id).interpolate();
  }
}

bool Interpolator::is_finished() const
{
  return state == END;
}

void Interpolator::next_pose()
{
  for (const auto & joint :
    actions[current_action_index].get_pose(current_pose_index).get_joints())
  {
    if (joint_processes.find(joint.get_id()) != joint_processes.end()) {
      joint_processes.at(joint.get_id()).set_target_position(joint.get_position());
    }
  }
  current_pose_index++;
}

bool Interpolator::check_for_next()
{
  int joint_number = joint_processes.size();
  for (auto [id, joint] : joint_processes) {
    if (joint.is_finished()) {
      joint_number--;
    }
  }

  return joint_number <= 0;
}

void Interpolator::change_state(int state)
{
  this->state = state;
  init_state = true;
}

std::vector<tachimawari::joint::Joint> Interpolator::get_joints() const
{
  std::vector<tachimawari::joint::Joint> joints;

  for (auto [id, joint] : joint_processes) {
    joints.push_back(joint);
  }

  return joints;
}

}  // namespace akushon
