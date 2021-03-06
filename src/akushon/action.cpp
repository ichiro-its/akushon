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

#include <akushon/action.hpp>

#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace akushon
{

Action::Action(const std::string & action_name)
: name(action_name), current_pose_index(0),
  pause_start_time(0), on_pause(false), on_process(false),
  is_start(false), start_stop_time(0)
{
}

void Action::insert_pose(const Pose & pose)
{
  poses.push_back(pose);
  pose_count = poses.size();
}

void Action::insert_pose(const uint8_t & id, const Pose & pose)
{
  poses.insert(poses.begin() + id, pose);
  pose_count = poses.size();
}

void Action::delete_pose(const uint8_t & id)
{
  poses.erase(poses.begin() + id);
  pose_count = poses.size();
}

void Action::load_data(const std::string & path)
{
  std::ifstream file(path);
  nlohmann::json action_data = nlohmann::json::parse(file);

  name = action_data["name"];
  for (auto &[key, val] : action_data.items()) {
    if (key.find("step_") != std::string::npos) {
      Pose pose(key);
      std::vector<tachimawari::Joint> joints;

      for (auto &[steps_key, steps_val] : action_data[key].items()) {
        if (!(steps_key.find("step_") != std::string::npos)) {
          tachimawari::Joint joint(steps_key, static_cast<float>(steps_val));  // init join
          joints.push_back(joint);
        } else if (steps_key == "step_pause") {
          pose.set_pause(static_cast<float>(steps_val));
        } else if (steps_key == "step_speed") {
          pose.set_speed(static_cast<float>(steps_val));
        }
      }

      pose.set_joints(joints);
      insert_pose(pose);
    }
  }
}

void Action::set_name(const std::string & action_name)
{
  name = action_name;
}

const std::string & Action::get_name() const
{
  return name;
}

const std::vector<Pose> & Action::get_poses() const
{
  return poses;
}

const Pose & Action::get_current_pose() const
{
  return poses.at(current_pose_index);
}

const Pose & Action::get_pose_by_index(const uint8_t & id) const
{
  return poses.at(id);
}

void Action::next_pose()
{
  current_pose_index++;
}

bool Action::is_running() const
{
  return on_process || is_start;
}

Pose Action::process(Pose robot_pose, const int & time)
{
  if (!is_start && !on_process) {
    start_stop_time = time;
    is_start = true;
  }

  if (time - start_stop_time > 1000 && is_start) {
    auto target_pose = get_current_pose();

    if (!on_process) {
      on_process = true;
      robot_pose.set_target_position(get_current_pose());
    }

    if (robot_pose == target_pose) {
      if (!on_pause) {
        pause_start_time = time;
        on_pause = true;
      }

      if (time - pause_start_time >= get_current_pose().get_pause() * 1000) {
        next_pose();
        on_pause = false;

        if (current_pose_index == pose_count) {
          start_stop_time = time;
          is_start = false;
        } else {
          robot_pose.set_target_position(get_current_pose());
        }
      }
    }

    if (!on_pause) {
      robot_pose.interpolate();
    }
  } else if (time - start_stop_time > 2000) {
    on_process = false;
    current_pose_index = 0;
  }

  return robot_pose;
}

void Action::reset()
{
  current_pose_index = 0;
  pause_start_time = 0;
  on_pause = false;
  on_process = false;
}

}  // namespace akushon
