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

#ifndef AKUSHON__ACTION__NODE__ACTION_NODE_HPP_
#define AKUSHON__ACTION__NODE__ACTION_NODE_HPP_

#include <memory>
#include <string>

#include "akushon/action/model/action.hpp"
#include "akushon/action/model/pose.hpp"
#include "akushon/action/node/action_manager.hpp"
#include "akushon_interfaces/msg/run_action.hpp"
#include "akushon_interfaces/msg/status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tachimawari_interfaces/msg/current_joints.hpp"
#include "tachimawari_interfaces/msg/set_joints.hpp"
#include "keisan/keisan.hpp"

namespace akushon
{

class ActionNode
{
public:
  using CurrentJoints = tachimawari_interfaces::msg::CurrentJoints;
  using Empty = std_msgs::msg::Empty;
  using RunAction = akushon_interfaces::msg::RunAction;
  using SetJoints = tachimawari_interfaces::msg::SetJoints;
  using Status = akushon_interfaces::msg::Status;
  using Float64 = std_msgs::msg::Float64;

  enum { READY, PLAYING };

  enum { RUN_ACTION_BY_NAME, RUN_ACTION_BY_JSON };

  static std::string get_node_prefix();
  static std::string run_action_topic();
  static std::string brake_action_topic();
  static std::string status_topic();
  static std::string ball_topic();

  explicit ActionNode(
    rclcpp::Node::SharedPtr node, std::shared_ptr<ActionManager> & action_manager);

  bool start(const std::string & action_name);
  bool start(const Action & action);

  bool update(int time);

private:
  void publish_joints();
  void publish_status();

  Pose initial_pose;
  double ball_pos;

  rclcpp::Node::SharedPtr node;

  std::shared_ptr<ActionManager> action_manager;

  rclcpp::Subscription<CurrentJoints>::SharedPtr current_joints_subscriber;
  rclcpp::Publisher<SetJoints>::SharedPtr set_joints_publisher;

  rclcpp::Subscription<RunAction>::SharedPtr run_action_subscriber;
  rclcpp::Subscription<Empty>::SharedPtr brake_action_subscriber;
  rclcpp::Subscription<Float64>::SharedPtr ball_subscriber;
  rclcpp::Publisher<Status>::SharedPtr status_publisher;
};

}  // namespace akushon

#endif  // AKUSHON__ACTION__NODE__ACTION_NODE_HPP_
