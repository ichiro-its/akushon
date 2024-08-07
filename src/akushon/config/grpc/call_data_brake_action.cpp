// Copyright (c) 2024 Ichiro ITS
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

#include "akushon/config/grpc/call_data_brake_action.hpp"
#include "akushon/config/utils/config.hpp"

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

namespace akushon
{

CallDataBrakeAction::CallDataBrakeAction(
  akushon_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string& path, const std::shared_ptr<ActionManager>& action_manager)
: CallData(service, cq, path), action_manager_(action_manager)
{
  Proceed();
}

void CallDataBrakeAction::AddNextToCompletionQueue()
{
  new CallDataBrakeAction(service_, cq_, path_, action_manager_);
}

void CallDataBrakeAction::WaitForRequest()
{
  service_->RequestBrakeAction(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void CallDataBrakeAction::HandleRequest()
{
  try {
    action_manager_->brake();
    RCLCPP_INFO(rclcpp::get_logger("BrakeAction"), "Action has been braked!");
  } catch (nlohmann::json::exception e) {
    RCLCPP_ERROR(rclcpp::get_logger("BrakeAction"), e.what());
  }
}

} // namespace akushon
