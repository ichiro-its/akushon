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

#include <stdlib.h>
#include <unistd.h>

#include <akushon/action_manager.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tachimawari_interfaces/srv/set_joints.hpp>
#include <tachimawari_interfaces/msg/joint.hpp>
#include <tachimawari/joint.hpp>

#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace akushon
{

// ActionManager::ActionManager(std::string node_name, std::string service_name)
// : is_running(false), current_action(std::make_shared<Action>("current_action")),
//   robot_pose(std::make_shared<Pose>("robot_pose"))
// {
// }

ActionManager::ActionManager(std::vector<std::string> action_names)
: action_names(action_names), action_list(std::map<uint8_t, Action>()),
  current_action(nullptr), robot_pose(std::make_shared<Pose>("robot_pose"))
{
}

void ActionManager::insert_action(const uint8_t & id, const Action & action)
{
  action_list.insert({id, action});
}

void ActionManager::delete_action(const uint8_t & id)
{
  action_list.erase(id);
}

const Action & ActionManager::get_action_by_id(const uint8_t & id) const
{
  return action_list.at(id);
}

bool ActionManager::is_ready() const
{
  // if (!set_joints_client->wait_for_service()) {
  // RCLCPP_INFO(get_logger(), "service not available");
  //   return false;
  // }

  return true;
}

std::shared_ptr<Pose> ActionManager::run_action(const int & time)
{
  current_action->start(robot_pose, time);

  if (current_action->is_finished()) {
    current_action = nullptr;

    return robot_pose;
  }

  return robot_pose;
}

void ActionManager::set_current_action(const uint8_t & action_id, const Pose & pose)
{
  current_action = std::make_shared<Action>(action_list.at(action_id));
  robot_pose = std::make_shared<Pose>(pose);  // init pose
}

bool ActionManager::set_current_action(const std::string & action_name)
{
  return set_current_action(action_name, *robot_pose);
}

bool ActionManager::set_current_action(const std::string & action_name, const Pose & robot_pose)
{
  auto result = std::find(action_names.begin(), action_names.end(), action_name);

  if (result != action_names.end()) {
    set_current_action(result - action_names.begin(), robot_pose);
    return true;
  }

  return false;
}

void ActionManager::load_action_data(const std::string & path)
{
  load_action_data(path, action_names);
}

void ActionManager::load_action_data(
  const std::string & path,
  const std::vector<std::string> & action_names)
{
  clear_action_list();
  this->action_names = action_names;

  uint8_t id = 0;
  for (auto action_name : action_names) {
    std::string file_name = path + "/" + action_name + ".json";
    std::ifstream file(file_name);
    nlohmann::json action_data = nlohmann::json::parse(file);

    Action action(action_data["name"]);

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
        action.insert_pose(pose);
      }
    }

    action_list.insert(std::pair<uint8_t, Action>(id, action));
    id++;
  }
}

bool ActionManager::is_empty() const
{
  return action_list.empty();
}

bool ActionManager::is_running() const
{
  return current_action != nullptr;
}

void ActionManager::clear_current_action()
{
  current_action = nullptr;
}

void ActionManager::clear_action_list()
{
  current_action = nullptr;
  action_list.clear();
}

// std::shared_future<std::shared_ptr<tachimawari_interfaces::srv::SetJoints::Response>>
// ActionManager::send_joints_request(std::vector<tachimawari::Joint> joints, float speed)
// {
//   {
//     using SetJoints = tachimawari_interfaces::srv::SetJoints;

//     auto request = std::make_shared<SetJoints::Request>();
//     std::vector<tachimawari_interfaces::msg::Joint> joint_messages;

//     for (auto joint : joints) {
//       tachimawari_interfaces::msg::Joint joint_message;
//       joint_message.name = joint.get_joint_name();
//       joint_message.position = joint.get_goal_position();
//       joint_message.speed = speed;

//       joint_messages.push_back(joint_message);
//     }

//     request->joints = joint_messages;

//     return set_joints_client->async_send_request(request);
//   }
// }

}  // namespace akushon
