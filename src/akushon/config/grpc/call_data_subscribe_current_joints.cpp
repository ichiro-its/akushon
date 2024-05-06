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

#include "akushon/config/grpc/call_data_subscribe_current_joints.hpp"

#include <chrono>
#include <string>

#include "akushon/config/utils/config.hpp"
#include "nlohmann/json.hpp"

namespace akushon
{
CallDataSubscribeCurrentJoints::CallDataSubscribeCurrentJoints(
  akushon_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string& path, rclcpp::Node::SharedPtr& node)
: CallData(service, cq, path), node_(node)
{
  current_joint_subscription_ =
    node_->create_subscription<tachimawari_interfaces::msg::CurrentJoints>(
      "/joint/current_joints", 10,
      [this](const tachimawari_interfaces::msg::CurrentJoints::SharedPtr curr_joints) {          
        curr_joints_.joints = curr_joints->joints;
      });

  Proceed();
}  // namespace akushon

void CallDataSubscribeCurrentJoints::AddNextToCompletionQueue()
{
  new CallDataSubscribeCurrentJoints(service_, cq_, path_, node_);
}

void CallDataSubscribeCurrentJoints::WaitForRequest()
{
  service_->RequestSubscribeCurrentJoints(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void CallDataSubscribeCurrentJoints::HandleRequest()
{
  try {
    nlohmann::json curr_joints;
    
    for (const auto & items : curr_joints_.joints) {
      nlohmann::json obj;
      obj["id"] = std::to_string(items.id);
      obj["position"] = std::to_string(items.position);
      curr_joints.push_back(obj);
    }
    reply_.set_msg_joints(curr_joints.dump());
    RCLCPP_INFO(rclcpp::get_logger("PublishSetTorques"), "curr joints has been sended!");
    current_joint_subscription_.reset();
  } catch (nlohmann::json::exception e) {
    RCLCPP_ERROR(rclcpp::get_logger("PublishSetTorques"), e.what());
  }
}
}  // namespace akushon
