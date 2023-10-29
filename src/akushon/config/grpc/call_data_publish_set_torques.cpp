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

#include "akushon/config/grpc/call_data_publish_set_torques.hpp"
#include "akushon/config/utils/config.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nlohmann/json.hpp"

namespace akushon
{
CallDataPublishSetTorques::CallDataPublishSetTorques(
  akushon_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string path, rclcpp::Node::SharedPtr node)
: CallData(service, cq, path), node_(node)
{
  set_torque_publisher_ =
    node_->create_publisher<tachimawari_interfaces::msg::SetTorques>("/joint/set_torques", 10);
  Proceed();
}

void CallDataPublishSetTorques::AddNextToCompletionQueue()
{
  new CallDataPublishSetTorques(service_, cq_, path_, node_);
}

void CallDataPublishSetTorques::WaitForRequest()
{
  service_->RequestSetTorques(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void CallDataPublishSetTorques::HandleRequest()
{
  try {
    tachimawari_interfaces::msg::SetTorques set_torque;
    set_torque.torque_enable = request_.torque_enable();

    nlohmann::json publish_set_torque = nlohmann::json::parse(request_.ids());

    for (const auto & items : publish_set_torque["id"].items()) {
      set_torque.ids.push_back((uint8_t)items.value().get<uint8_t>());
    }
    set_torque_publisher_->publish(set_torque);
    RCLCPP_INFO(rclcpp::get_logger("PublishSetTorques"), "set torques has been published!");

  } catch (std::ofstream::failure f) {
    RCLCPP_ERROR(rclcpp::get_logger("PublishSetTorques"), f.what());

  } catch (nlohmann::json::exception e) {
    RCLCPP_ERROR(rclcpp::get_logger("PublishSetTorques"), e.what());
  }
}
} // namespace akushon
