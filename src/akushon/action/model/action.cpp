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

#include "akushon/action/model/action.hpp"

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "akushon/action/model/pose.hpp"
#include "tachimawari/joint/model/joint_id.hpp"

namespace akushon
{

Action::Action(const std::string & action_name)
: name(action_name), poses({}), start_delay(0), stop_delay(0), time_based(false), next_action("")
{
}

void Action::add_pose(const Pose & pose) { poses.push_back(pose); }

void Action::set_pose(int index, const Pose & pose) { poses.insert(poses.begin() + index, pose); }

void Action::delete_pose(int index) { poses.erase(poses.begin() + index); }

void Action::set_name(const std::string & action_name) { name = action_name; }

const std::string & Action::get_name() const { return name; }

const std::vector<Pose> & Action::get_poses() const { return poses; }

const Pose & Action::get_pose(int index) const { return poses.at(index); }

int Action::get_pose_count() const { return poses.size(); }

void Action::set_start_delay(int start_delay) { this->start_delay = start_delay; }

int Action::get_start_delay() const { return start_delay; }

void Action::set_stop_delay(int stop_delay) { this->stop_delay = stop_delay; }

int Action::get_stop_delay() const { return stop_delay; }

void Action::set_next_action(const std::string & next_action) { this->next_action = next_action; }

const std::string & Action::get_next_action() const { return next_action; }

void Action::set_joint_target_position(int pose_index, uint8_t joint_id, float target_position)
{
  poses.at(pose_index).set_target_position(joint_id, target_position);
}

void Action::reset() { poses.clear(); }

nlohmann::json Action::get_json()
{
  nlohmann::json json_action;

  json_action["name"] = get_name();
  json_action["next"] = get_next_action();

  for (const auto & pose : get_poses()) {
    nlohmann::json json_pose;
    nlohmann::json json_joints;

    using tachimawari::joint::JointId;

    for (const auto & joint : pose.get_joints()) {
      std::string joint_name = JointId::by_id.at(joint.get_id());
      json_joints[joint_name] = joint.get_position();
    }

    json_pose["joints"] = json_joints;
    json_pose["name"] = pose.get_name();
    json_pose["pause"] = pose.get_pause();
    json_pose["speed"] = pose.get_speed();
    json_pose["time"] = pose.action_time;

    json_action["poses"].push_back(json_pose);
  }

  json_action["start_delay"] = get_start_delay();
  json_action["stop_delay"] = get_stop_delay();
  json_action["time_based"] = time_based;

  return json_action;
}

}  // namespace akushon
