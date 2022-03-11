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
#include <thread>

#include "akushon/node/akushon_node.hpp"

#include "akushon/action/model/action.hpp"
#include "akushon/action/node/action_manager.hpp"
#include "akushon/action/node/action_node.hpp"
#include "akushon_interfaces/action/run_action.hpp"
#include "akushon_interfaces/srv/save_actions.hpp"
#include "akushon_interfaces/srv/get_actions.hpp"
#include "akushon_interfaces/srv/run_pose.hpp"
#include "akushon_interfaces/srv/run_action.hpp"
#include "tachimawari/joint/model/joint.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace akushon
{

AkushonNode::AkushonNode(rclcpp::Node::SharedPtr node)
: node(node), action_node(nullptr)
{
  {
    using akushon_interfaces::srv::RunAction;
    run_action_service = node->create_service<RunAction>(
      "/run_action",
      [this](std::shared_ptr<RunAction::Request> request,
      std::shared_ptr<RunAction::Response> response) {
        // TODO(finesaaa): need real test
        // response->status = AkushonNode::handle_run_action(request);

        // TODO(finesaaa): temporary for checking
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[RUN ACTION] Get request: " + request->json);
        response->status = "ACCEPTED";
      }
    );
  }

  {
    using akushon_interfaces::srv::SaveActions;
    save_actions_service = node->create_service<SaveActions>(
      "/save_actions",
      [this](std::shared_ptr<SaveActions::Request> request,
      std::shared_ptr<SaveActions::Response> response) {
        this->action_node->save_all_actions(request->json);
        response->status = "SAVED";
      }
    );
  }

  {
    using akushon_interfaces::srv::GetActions;
    get_actions_service = node->create_service<GetActions>(
      "/get_actions",
      [this](std::shared_ptr<GetActions::Request> request,
      std::shared_ptr<GetActions::Response> response) {
        response->json = this->action_node->get_all_actions();
      }
    );
  }

  {
    using akushon_interfaces::srv::RunPose;
    run_pose_service = node->create_service<RunPose>(
      "/run_pose",
      [this](std::shared_ptr<RunPose::Request> request,
      std::shared_ptr<RunPose::Response> response) {
        {
          using tachimawari::joint::Joint;

          std::vector<Joint> joints;
          for (const auto & joint : request->joints) {
            joints.push_back(Joint(joint.id, joint.position));
          }

          this->action_node->publish_joints(joints);
          response->status = "PUBLISHED";
        }
      }
    );
  }


  run_action_server = rclcpp_action::create_server<RunAction>(
    node->get_node_base_interface(),
    node->get_node_clock_interface(),
    node->get_node_logging_interface(),
    node->get_node_waitables_interface(),
    "run_action",
    std::bind(&AkushonNode::handle_goal, this, _1, _2),
    std::bind(&AkushonNode::handle_cancel, this, _1),
    std::bind(&AkushonNode::handle_accepted, this, _1));
}

std::string AkushonNode::handle_run_action(std::shared_ptr<akushon_interfaces::srv::RunAction::Request> request) 
{
  rclcpp::Rate rcl_rate(8ms);

  bool is_ready = false;
  Action action = action_node->load_json_action(request->json);
  is_ready = action_node->start(action); 

  if (is_ready) {
    while (rclcpp::ok()) {
      rcl_rate.sleep();

      if (action_node->get_status() == ActionNode::PLAYING) {
        action_node->process(this->node->now().seconds() * 1000);
      } else if (action_node->get_status() == ActionNode::READY) {
        break;
      }
    }
  }

  if (rclcpp::ok()) {
    return is_ready? "SUCCEEDED" : "FAILED";
  }
}

rclcpp_action::GoalResponse AkushonNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const RunAction::Goal> goal)
{
  bool is_action_exist = false;

  if (action_node->get_status() == ActionNode::READY && action_node) {
    if (goal->action_id >= 0) {
      is_action_exist = action_node->is_action_exist(goal->action_id);
    } else if (goal->action_name != "") {
      is_action_exist = action_node->is_action_exist(goal->action_name);
    }
  }

  return is_action_exist ?
         rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
         rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse AkushonNode::handle_cancel(
  const std::shared_ptr<GoalHandleRunAction> goal_handle)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void AkushonNode::handle_accepted(const std::shared_ptr<GoalHandleRunAction> goal_handle)
{
  std::thread{[this](const std::shared_ptr<GoalHandleRunAction> goal_handle) {
      rclcpp::Rate rcl_rate(8ms);

      const auto goal = goal_handle->get_goal();

      bool is_ready = false;
      if (goal->action_id >= 0) {
        is_ready = action_node->start(goal->action_id);
      } else if (goal->action_name != "") {
        is_ready = action_node->start(goal->action_name);
      }

      auto feedback = std::make_shared<RunAction::Feedback>();
      auto result = std::make_shared<RunAction::Result>();

      if (is_ready) {
        while (rclcpp::ok()) {
          rcl_rate.sleep();

          if (goal_handle->is_canceling()) {
            result->status = CANCELED;
            goal_handle->canceled(result);
            return;
          }

          if (action_node->get_status() == ActionNode::PLAYING) {
            action_node->process(this->node->now().seconds() * 1000);
          } else if (action_node->get_status() == ActionNode::READY) {
            break;
          }

          goal_handle->publish_feedback(feedback);
        }
      }

      if (rclcpp::ok()) {
        result->status = is_ready ? SUCCEEDED : FAILED;
        goal_handle->succeed(result);
      }
    }, goal_handle}.join();
}

void AkushonNode::set_action_manager(std::shared_ptr<ActionManager> action_manager)
{
  action_node = std::make_shared<ActionNode>(node, action_manager);
}

}  // namespace akushon
