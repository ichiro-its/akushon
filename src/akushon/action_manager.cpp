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

#include <rclcpp/rclcpp.hpp>
#include <tachimawari_interfaces/srv/set_joints.hpp>
#include <tachimawari_interfaces/msg/joint.hpp>
#include <tachimawari/joint.hpp>

#include <akushon/action_manager.hpp>

#include <string>
#include <memory>
#include <vector>

namespace akushon
{

ActionManager::ActionManager(std::string node_name, std::string service_name)
: rclcpp::Node(node_name)
{
  {
    using SetJoints = tachimawari_interfaces::srv::SetJoints;
    set_joints_client = this->create_client<SetJoints>(service_name + "/set_joints");
  }
}

void ActionManager::insert_action(uint8_t id, std::shared_ptr<Action> action)
{
  action_list.insert({id, action});
}

void ActionManager::delete_action(uint8_t id)
{
  action_list.erase(id);
}

std::shared_ptr<Action> ActionManager::get_action(uint8_t id)
{
  return action_list[id];
}

std::shared_future<std::shared_ptr<tachimawari_interfaces::srv::SetJoints::Response>>
ActionManager::send_joints_request(std::vector<tachimawari::Joint> joints, float speed)
{
  {
    using SetJoints = tachimawari_interfaces::srv::SetJoints;

    auto request = std::make_shared<SetJoints::Request>();
    std::vector<tachimawari_interfaces::msg::Joint> joint_messages;

    for (auto joint : joints) {
      tachimawari_interfaces::msg::Joint joint_message;
      joint_message.name = joint.get_joint_name();
      joint_message.position = joint.get_goal_position();
      joint_message.speed = speed;

      joint_messages.push_back(joint_message);
    }

    request->joints = joint_messages;

    return set_joints_client->async_send_request(request);
  }
}

}  // namespace akushon
