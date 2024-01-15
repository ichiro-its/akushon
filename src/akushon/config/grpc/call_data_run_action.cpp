// Copyright (c) 2023 Ichiro ITS
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

#include "akushon/config/grpc/call_data_run_action.hpp"
#include "akushon/config/utils/config.hpp"
#include "rclcpp/rclcpp.hpp"

namespace akushon
{
CallDataRunAction::CallDataRunAction(
  akushon_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string& path, rclcpp::Node::SharedPtr& node)
: CallData(service, cq, path), node_(node)
{
  run_action_publisher_ =
    node_->create_publisher<akushon_interfaces::msg::RunAction>("/action/run_action", 10);
  Proceed();
}

void CallDataRunAction::AddNextToCompletionQueue()
{
  new CallDataRunAction(service_, cq_, path_, node_);
}

void CallDataRunAction::WaitForRequest()
{
  service_->RequestRunAction(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void CallDataRunAction::HandleRequest()
{
  akushon_interfaces::msg::RunAction run_action;
  run_action.control_type = request_.control_type();
  run_action.action_name = request_.action_name();
  run_action.json = request_.json_action();
  run_action_publisher_->publish(run_action);
  RCLCPP_INFO(rclcpp::get_logger("PublishSetJoints"), "run action config has been published!");
}
} // namespace akushon
