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

#include "akushon/config/grpc/call_data_publish_set_joints.hpp"
#include "akushon/config/utils/config.hpp"
#include "nlohmann/json.hpp"

namespace akushon
{
CallDataPublishSetJoints::CallDataPublishSetJoints(
  akushon_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string& path, const rclcpp::Node::SharedPtr& node)
: CallData(service, cq, path), node_(node)
{
  set_joints_publisher_ =
    node_->create_publisher<tachimawari_interfaces::msg::SetJoints>("/joint/set_joints", 10);
  Proceed();
}

void CallDataPublishSetJoints::AddNextToCompletionQueue()
{
  new CallDataPublishSetJoints(service_, cq_, path_, node_);
}

void CallDataPublishSetJoints::WaitForRequest()
{
  service_->RequestPublishSetJoints(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void CallDataPublishSetJoints::HandleRequest()
{
  try {
    int control_type = request_.control_type();
    tachimawari_interfaces::msg::SetJoints set_joints_;
    set_joints_.control_type = control_type;
    // set_joints_.joints.push_back(joint_actions);
    nlohmann::json publish_set_joint = nlohmann::json::parse(request_.joints_actions());
    for (const auto & items_json_ : publish_set_joint["joints"].items()) {
      nlohmann::json obj = items_json_.value();
      tachimawari_interfaces::msg::Joint joint;
      joint.id = (uint8_t)obj.at("id").get<uint8_t>();
      joint.position = (float)obj.at("position").get<float>();
      set_joints_.joints.push_back(joint);
    }
    set_joints_publisher_->publish(set_joints_);
    
    RCLCPP_INFO(rclcpp::get_logger("PublishSetJoints"), "set joints has been published!");
  } catch (nlohmann::json::exception e) {
    RCLCPP_ERROR(rclcpp::get_logger("PublishSetJoints"), e.what());
  }
}
} // namespace akushon
