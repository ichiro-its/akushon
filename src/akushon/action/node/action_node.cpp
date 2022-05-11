// Copyright (c) 2021 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <chrono>
#include <iostream>
#include <iomanip>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "akushon/action/node/action_node.hpp"

#include "akushon/action/node/action_manager.hpp"
#include "akushon/action/model/action_name.hpp"
#include "akushon/action/model/pose.hpp"
#include "akushon_interfaces/srv/run_action.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tachimawari/joint/model/joint.hpp"
#include "tachimawari_interfaces/msg/set_joints.hpp"

using namespace std::chrono_literals;

namespace akushon
{

ActionNode::ActionNode(
  rclcpp::Node::SharedPtr node, std::shared_ptr<ActionManager> action_manager)
: node(node), action_manager(action_manager), status(READY), now(node->now().seconds()),
  initial_pose(Pose("initial_pose"))
{
  current_joints_subscriber = node->create_subscription<tachimawari_interfaces::msg::CurrentJoints>(
    "/joint/current_joints", 10,
    [this](const tachimawari_interfaces::msg::CurrentJoints::SharedPtr message) {
      {
        using tachimawari::joint::Joint;
        std::vector<Joint> current_joints;

        for (const auto & joint : message->joints) {
          current_joints.push_back(Joint(joint.id, joint.position));
        }

        this->initial_pose.set_joints(current_joints);
      }
    }
  );

  set_joints_publisher = node->create_publisher<tachimawari_interfaces::msg::SetJoints>(
    "/joint/set_joints", 10);

  {
    using akushon_interfaces::srv::RunAction;
    run_action_service = node->create_service<RunAction>(
      get_node_prefix() + "/run_action",
      [this](std::shared_ptr<RunAction::Request> request,
      std::shared_ptr<RunAction::Response> response) {
        rclcpp::Rate rcl_rate(8ms);

        nlohmann::json action_data = nlohmann::json::parse(request->json);
        Action action = this->action_manager->load_action(action_data, "temp_action");
        bool is_ready = start(action);

        if (is_ready) {
          while (rclcpp::ok()) {
            rcl_rate.sleep();

            if (get_status() == ActionNode::PLAYING) {
              double time = this->node->now().seconds() - this->now;
              process(time * 1000);
            } else if (get_status() == ActionNode::READY) {
              break;
            }
          }
        }

        if (rclcpp::ok()) {
          response->status = is_ready ? "SUCCEEDED" : "FAILED";
        }
      }
    );
  }

  {
    using akushon_interfaces::srv::RunAction;
    run_action_pkg_service = node->create_service<RunAction>(
      get_node_prefix() + "/run_action_pkg",
      [this](std::shared_ptr<RunAction::Request> request,
      std::shared_ptr<RunAction::Response> response) {
        rclcpp::Rate rcl_rate(8ms);

        nlohmann::json action_name = nlohmann::json::parse(request->json);
        bool is_ready = start(action_name.dump());

        if (is_ready) {
          while (rclcpp::ok()) {
            rcl_rate.sleep();

            if (get_status() == ActionNode::PLAYING) {
              double time = this->node->now().seconds() - this->now;
              process(time * 1000);
            } else if (get_status() == ActionNode::READY) {
              break;
            }
          }
        }

        if (rclcpp::ok()) {
          response->status = is_ready ? "SUCCEEDED" : "FAILED";
        }
      }
    );
  }
}

bool ActionNode::is_action_exist(const std::string & action_name) const
{
  return action_manager->get_action(action_name).get_name().empty();
}

int ActionNode::get_status() const
{
  return status;
}

bool ActionNode::start(const std::string & action_name)
{
  Pose pose = this->initial_pose;

  if (!pose.get_joints().empty()) {
    action_manager->start(action_name, pose);
    status = PLAYING;
  } else {
    // Failed to call service
    return false;
  }

  return true;
}

bool ActionNode::start(const Action & action)
{
  Pose pose = this->initial_pose;

  if (!pose.get_joints().empty()) {
    action_manager->start(action, pose);
    status = PLAYING;
  } else {
    // Failed to call service
    return false;
  }

  return true;
}

void ActionNode::process(int time)
{
  action_manager->process(time);

  if (action_manager->is_playing()) {
    publish_joints();
  } else {
    status = READY;
  }
}

std::string ActionNode::get_node_prefix() const
{
  return "action";
}

void ActionNode::publish_joints()
{
  auto joints_msg = tachimawari_interfaces::msg::SetJoints();

  const auto & joints = action_manager->get_joints();
  auto & joint_msgs = joints_msg.joints;

  joint_msgs.resize(joints.size());
  for (size_t i = 0; i < joints.size() && i < joint_msgs.size(); ++i) {
    joint_msgs[i].id = joints[i].get_id();
    joint_msgs[i].position = joints[i].get_position();
  }

  set_joints_publisher->publish(joints_msg);
}

}  // namespace akushon
