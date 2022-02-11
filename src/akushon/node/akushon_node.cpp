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

#include <memory>
#include <string>

#include "akushon/node/akushon_node.hpp"

#include "akushon/action/node/action_manager.hpp"
#include "akushon/action/node/action_node.hpp"
#include "akushon_interfaces/action/run_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace akushon
{

AkushonNode::AkushonNode(rclcpp::Node::SharedPtr node)
: node(node)
{
  {
    using akushon_interfaces::action::RunAction;
    using GoalHandleRunAction = rclcpp_action::ServerGoalHandle<RunAction>;

    run_action_server = rclcpp_action::create_server<RunAction>(
      node->get_node_base_interface(),
      node->get_node_clock_interface(),
      node->get_node_logging_interface(),
      node->get_node_waitables_interface(),
      "run_action",
      [this] (const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const RunAction::Goal> goal) -> rclcpp_action::GoalResponse {
        bool is_action_exist = false;

        if (action_node->get_status() == ActionNode::READY) {
          if (goal->action_id >= 0) {
            is_action_exist = action_node->is_action_exist(goal->action_id);
          } else if (goal->action_name != "") {
            is_action_exist = action_node->is_action_exist(goal->action_name);
          }
        }

        if (is_action_exist) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        } else {
          return rclcpp_action::GoalResponse::REJECT;
        }
      },
      [this] (const std::shared_ptr<GoalHandleRunAction> goal_handle) -> rclcpp_action::CancelResponse {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this] (const std::shared_ptr<GoalHandleRunAction> goal_handle) {
        
      });
  }
}

void AkushonNode::set_action_manager(std::shared_ptr<ActionManager> action_manager)
{
  action_node = std::make_shared<ActionNode>(node, action_manager);
}

}  // namespace akushon
