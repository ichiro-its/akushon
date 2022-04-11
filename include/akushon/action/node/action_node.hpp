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

#ifndef AKUSHON__ACTION__NODE__ACTION_NODE_HPP_
#define AKUSHON__ACTION__NODE__ACTION_NODE_HPP_

#include <memory>
#include <string>

#include "akushon/action/model/action.hpp"
#include "akushon/action/model/pose.hpp"
#include "akushon/action/node/action_manager.hpp"
#include "akushon_interfaces/srv/run_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tachimawari_interfaces/msg/current_joints.hpp"
#include "tachimawari_interfaces/msg/set_joints.hpp"
#include "tachimawari_interfaces/srv/get_joints.hpp"

namespace akushon
{

class ActionNode
{
public:
  enum
  {
    READY,
    PLAYING
  };

  explicit ActionNode(
    rclcpp::Node::SharedPtr node, std::shared_ptr<ActionManager> action_manager);

  bool is_action_exist(int action_id) const;
  bool is_action_exist(const std::string & action_name) const;

  bool start(const std::string & action_name);
  bool start(int action_id);
  bool start(const Action & action);

  void process(int time);

  int get_status() const;

private:
  std::string handle_run_action(
    std::shared_ptr<akushon_interfaces::srv::RunAction::Request> request);

  std::string get_node_prefix() const;

  void publish_joints();

  rclcpp::Node::SharedPtr node;

  std::shared_ptr<ActionManager> action_manager;

  rclcpp::Subscription<tachimawari_interfaces::msg::CurrentJoints>::SharedPtr
    current_joints_subscriber;
  rclcpp::Publisher<tachimawari_interfaces::msg::SetJoints>::SharedPtr set_joints_publisher;

  rclcpp::Service<akushon_interfaces::srv::RunAction>::SharedPtr run_action_service;

  int status;
  double now;
  Pose initial_pose;
};

}  // namespace akushon

#endif  // AKUSHON__ACTION__NODE__ACTION_NODE_HPP_
