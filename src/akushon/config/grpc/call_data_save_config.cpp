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

#include "akushon/config/grpc/call_data_save_config.hpp"
#include "akushon/config/utils/config.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nlohmann/json.hpp"

namespace akushon
{

CallDataSaveConfig::CallDataSaveConfig(
  akushon_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string& path)
: CallData(service, cq, path)
{
  Proceed();
}

void CallDataSaveConfig::AddNextToCompletionQueue()
{
  new CallDataSaveConfig(service_, cq_, path_);
}

void CallDataSaveConfig::WaitForRequest()
{
  service_->RequestSaveConfig(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void CallDataSaveConfig::HandleRequest()
{
  Config config(path_);
  try {
    config.save_config(request_.json_actions());
    RCLCPP_INFO(rclcpp::get_logger("SaveConfig"), "config has been saved!");
  } catch (nlohmann::json::exception e) {
    RCLCPP_ERROR(rclcpp::get_logger("SaveConfig"), e.what());
  }
}

} // namespace akushon
