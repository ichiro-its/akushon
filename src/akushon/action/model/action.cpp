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

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "akushon/action/model/action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "akushon/action/model/pose.hpp"

namespace akushon
{

Action::Action(const std::string & action_name)
: name(action_name), poses({}), start_delay(0), stop_delay(0), time_based(false), next_action("")
{
}

void Action::add_pose(const Pose & pose)
{
  poses.push_back(pose);
}

void Action::set_pose(int index, const Pose & pose)
{
  poses.insert(poses.begin() + index, pose);
}

void Action::delete_pose(int index)
{
  poses.erase(poses.begin() + index);
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

const Pose & Action::get_pose(int index) const
{
  return poses.at(index);
}

int Action::get_pose_count() const
{
  return poses.size();
}

void Action::set_start_delay(int start_delay)
{
  this->start_delay = start_delay;
}

int Action::get_start_delay() const
{
  return start_delay;
}

void Action::set_stop_delay(int stop_delay)
{
  this->stop_delay = stop_delay;
}

int Action::get_stop_delay() const
{
  return stop_delay;
}

void Action::set_next_action(const std::string & next_action)
{
  this->next_action = next_action;
}

const std::string & Action::get_next_action() const
{
  return next_action;
}

void Action::reset()
{
  poses.clear();
}

void Action::map_action(const Action & source_action, const Action & target_action, int target_pose_index, float source_val, float source_min, float source_max)
{
  std::vector<tachimawari::joint::Joint> src_joints = source_action.get_pose(target_pose_index).get_joints();
  std::vector<tachimawari::joint::Joint> target_joints = target_action.get_pose(target_pose_index).get_joints();
  for (int joint = src_joints.size() - 1; joint >= 0; --joint)
  {
      int joint_id = src_joints.at(joint).get_id();
      float target_min = src_joints.at(joint).get_position_value();
      float target_max = target_joints.at(joint).get_position_value();
      float new_joint_value = keisan::map(source_val, source_min, source_max, target_min, target_max);
      new_joint_value = keisan::curve(new_joint_value, target_min, target_max, float(2.0));
      new_joint_value = keisan::clamp(new_joint_value, target_min, target_max);
      target_joints[joint].set_position_value(static_cast<int>(std::round(new_joint_value)));
      RCLCPP_INFO(rclcpp::get_logger("Map Action"), "Joint %d: %f -> %d", joint_id, target_min, int(std::round(new_joint_value)));
      RCLCPP_INFO(rclcpp::get_logger("Map Action"), "Cek value in target joints %d: %f -> %d", target_joints[joint].get_id(), target_max, target_joints[joint].get_position_value());
  }
  Pose new_pose(target_action.get_pose(target_pose_index).get_name());
  new_pose.set_pause(source_action.get_pose(target_pose_index).get_pause());
  new_pose.set_speed(source_action.get_pose(target_pose_index).get_speed());
  new_pose.action_time = source_action.get_pose(target_pose_index).action_time;
  new_pose.set_joints(target_joints);
  
  RCLCPP_INFO(rclcpp::get_logger("Map Action"), "Cek new pose name %s", new_pose.get_name().c_str());
  RCLCPP_INFO(rclcpp::get_logger("Map Action"), "Cek new pose pause %f", new_pose.get_pause());
  RCLCPP_INFO(rclcpp::get_logger("Map Action"), "Cek new pose speed %f", new_pose.get_speed());
  RCLCPP_INFO(rclcpp::get_logger("Map Action"), "Cek new pose joints size %ld", new_pose.get_joints().size());
  for (int joint = new_pose.get_joints().size() - 1; joint >= 0; --joint)
  {
    RCLCPP_INFO(rclcpp::get_logger("Map Action"), "Cek new pose joints %d: %d", new_pose.get_joints().at(joint).get_id(), new_pose.get_joints().at(joint).get_position_value());
  }

  this->delete_pose(target_pose_index);
  this->set_pose(target_pose_index, new_pose);

  RCLCPP_INFO(rclcpp::get_logger("Map Action"), "Cek changed pose name %s", this->get_pose(target_pose_index).get_name().c_str());
  RCLCPP_INFO(rclcpp::get_logger("Map Action"), "Cek changed pose pause %f", this->get_pose(target_pose_index).get_pause());
  RCLCPP_INFO(rclcpp::get_logger("Map Action"), "Cek changed pose speed %f", this->get_pose(target_pose_index).get_speed());
  RCLCPP_INFO(rclcpp::get_logger("Map Action"), "Cek changed pose joints size %ld", this->get_pose(target_pose_index).get_joints().size());
  for (int joint = this->get_pose(target_pose_index).get_joints().size() - 1; joint >= 0; --joint)
  {
    RCLCPP_INFO(rclcpp::get_logger("Map Action"), "Cek changed pose joints %d: %d", this->get_pose(target_pose_index).get_joints().at(joint).get_id(), this->get_pose(target_pose_index).get_joints().at(joint).get_position_value());
  }
}

}  // namespace akushon
