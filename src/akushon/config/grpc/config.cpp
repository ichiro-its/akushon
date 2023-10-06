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

#include "akushon/config/grpc/config.hpp"

#include <chrono>
#include <csignal>
#include <future>
#include <string>

#include "akushon/config/utils/config.hpp"
#include "rclcpp/rclcpp.hpp"

using grpc::ServerBuilder;
using namespace std::chrono_literals;

namespace akushon
{
ConfigGrpc::ConfigGrpc() {}
ConfigGrpc::ConfigGrpc(const std::string & path) : path(path) {}

ConfigGrpc::~ConfigGrpc()
{
  server_->Shutdown();
  cq_->Shutdown();
}

void ConfigGrpc::SignIntHandler(int signum)
{
  server_->Shutdown();
  cq_->Shutdown();
  exit(signum);
}

void ConfigGrpc::Run(uint16_t port, const std::string path, rclcpp::Node::SharedPtr node)
{
  std::string server_address = absl::StrFormat("0.0.0.0:%d", port);

  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service_);

  cq_ = builder.AddCompletionQueue();
  server_ = builder.BuildAndStart();
  std::cout << "Server listening on " << server_address << std::endl;

  signal(SIGINT, SignIntHandler);
  async_server = std::thread([&path, this, &node]() {
    new ConfigGrpc::CallDataGetConfig(&service_, cq_.get(), path);
    new ConfigGrpc::CallDataSaveConfig(&service_, cq_.get(), path);
    new ConfigGrpc::CallDataPublishSetJoints(&service_, cq_.get(), path, node);
    new ConfigGrpc::CallDataPublishSetTorques(&service_, cq_.get(), path, node);
    new ConfigGrpc::CallDataRunAction(&service_, cq_.get(), path, node);
    new ConfigGrpc::CallDataSubscribeCurrentJoints(&service_, cq_.get(), path, node);
    void * tag;  // uniquely identifies a request.
    bool ok = true;
    while (true) {
      this->cq_->Next(&tag, &ok);
      if (ok) {
        static_cast<ConfigGrpc::CallDataBase *>(tag)->Proceed();
      }
    }
  });
  std::this_thread::sleep_for(200ms);
}

ConfigGrpc::CallDataBase::CallDataBase() {}

template <class ConfigRequest, class ConfigReply>
ConfigGrpc::CallData<ConfigRequest, ConfigReply>::CallData(
  akushon_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string path)
: status_(CREATE), service_(service), cq_(cq), responder_(&ctx_), path_(path)
{
}

template <class ConfigRequest, class ConfigReply>
void ConfigGrpc::CallData<ConfigRequest, ConfigReply>::Proceed()
{
  if (status_ == CREATE) {
    status_ = PROCESS;
    WaitForRequest();
  } else if (status_ == PROCESS) {
    AddNextToCompletionQueue();
    HandleRequest();
    status_ = FINISH;
    responder_.Finish(reply_, grpc::Status::OK, this);
  } else {
    GPR_ASSERT(status_ == FINISH);
    delete this;
  }
}

ConfigGrpc::CallDataGetConfig::CallDataGetConfig(
  akushon_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string path)
: CallData(service, cq, path)
{
  Proceed();
}

void ConfigGrpc::CallDataGetConfig::AddNextToCompletionQueue()
{
  new ConfigGrpc::CallDataGetConfig(service_, cq_, path_);
}

void ConfigGrpc::CallDataGetConfig::WaitForRequest()
{
  service_->RequestGetConfig(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void ConfigGrpc::CallDataGetConfig::HandleRequest()
{
  Config config(path_);

  reply_.set_json_actions(config.get_config());
  RCLCPP_INFO(rclcpp::get_logger("GetConfig"), "config has been sent!");
}

ConfigGrpc::CallDataSaveConfig::CallDataSaveConfig(
  akushon_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string path)
: CallData(service, cq, path)
{
  Proceed();
}

void ConfigGrpc::CallDataSaveConfig::AddNextToCompletionQueue()
{
  new ConfigGrpc::CallDataSaveConfig(service_, cq_, path_);
}

void ConfigGrpc::CallDataSaveConfig::WaitForRequest()
{
  service_->RequestSaveConfig(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void ConfigGrpc::CallDataSaveConfig::HandleRequest()
{
  Config config(path_);
  try {
    nlohmann::json akushon_data = nlohmann::json::parse(request_.json_actions());

    config.save_config(akushon_data);
    RCLCPP_INFO(rclcpp::get_logger("SaveConfig"), "config has been saved!  ");

  } catch (std::ofstream::failure f) {
    RCLCPP_ERROR(rclcpp::get_logger("SaveConfig"), f.what());

  } catch (nlohmann::json::exception e) {
    RCLCPP_ERROR(rclcpp::get_logger("SaveConfig"), e.what());
  }
}

ConfigGrpc::CallDataPublishSetJoints::CallDataPublishSetJoints(
  akushon_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string path, rclcpp::Node::SharedPtr node)
: CallData(service, cq, path), node_(node)
{
  set_joints_publisher_ =
    node_->create_publisher<tachimawari_interfaces::msg::SetJoints>("/joint/set_joints", 10);
  Proceed();
}

void ConfigGrpc::CallDataPublishSetJoints::AddNextToCompletionQueue()
{
  new ConfigGrpc::CallDataPublishSetJoints(service_, cq_, path_, node_);
}

void ConfigGrpc::CallDataPublishSetJoints::WaitForRequest()
{
  service_->RequestPublishSetJoints(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void ConfigGrpc::CallDataPublishSetJoints::HandleRequest()
{
  try {
    int control_type = request_.control_type();
    tachimawari_interfaces::msg::SetJoints set_joints_;
    set_joints_.control_type = control_type;
    // set_joints_.joints.push_back(joint_actions);
    nlohmann::json publish_set_joint = nlohmann::json::parse(request_.joints_actions());
    for (const auto & items_json_ : publish_set_joint.items()) {
      nlohmann::json obj = items_json_.value();
      tachimawari_interfaces::msg::Joint joint;
      joint.id = (uint8_t)atoi(obj.at("id").get<std::string>().c_str());
      joint.position = (float)atoi(obj.at("position").get<std::string>().c_str());
      set_joints_.joints.push_back(joint);
    }
    set_joints_publisher_->publish(set_joints_);

    //   config.save_config(akushon_data);
    RCLCPP_INFO(rclcpp::get_logger("PublishSetJoints"), "set joints has been published!  ");

  } catch (std::ofstream::failure f) {
    RCLCPP_ERROR(rclcpp::get_logger("PublishSetJoints"), f.what());

  } catch (nlohmann::json::exception e) {
    RCLCPP_ERROR(rclcpp::get_logger("PublishSetJoints"), e.what());
  }
}

ConfigGrpc::CallDataRunAction::CallDataRunAction(
  akushon_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string path, rclcpp::Node::SharedPtr node)
: CallData(service, cq, path), node_(node)
{
  run_action_publisher_ =
    node_->create_publisher<akushon_interfaces::msg::RunAction>("/action/run_action", 10);
  Proceed();
}

void ConfigGrpc::CallDataRunAction::AddNextToCompletionQueue()
{
  new ConfigGrpc::CallDataRunAction(service_, cq_, path_, node_);
}

void ConfigGrpc::CallDataRunAction::WaitForRequest()
{
  service_->RequestRunAction(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void ConfigGrpc::CallDataRunAction::HandleRequest()
{
  akushon_interfaces::msg::RunAction run_action;
  run_action.control_type = request_.control_type();
  run_action.action_name = request_.action_name();
  run_action.json = request_.json_action();
  run_action_publisher_->publish(run_action);
  RCLCPP_INFO(rclcpp::get_logger("PublishSetJoints"), "run action config has been published!");

}

ConfigGrpc::CallDataPublishSetTorques::CallDataPublishSetTorques(
  akushon_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string path, rclcpp::Node::SharedPtr node)
: CallData(service, cq, path), node_(node)
{
  set_torque_publisher_ =
    node_->create_publisher<tachimawari_interfaces::msg::SetTorques>("/joint/set_torques", 10);
  Proceed();
}

void ConfigGrpc::CallDataPublishSetTorques::AddNextToCompletionQueue()
{
  new ConfigGrpc::CallDataPublishSetTorques(service_, cq_, path_, node_);
}

void ConfigGrpc::CallDataPublishSetTorques::WaitForRequest()
{
  service_->RequestSetTorques(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void ConfigGrpc::CallDataPublishSetTorques::HandleRequest()
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

ConfigGrpc::CallDataSubscribeCurrentJoints::CallDataSubscribeCurrentJoints(
  akushon_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string path, rclcpp::Node::SharedPtr node)
: CallData(service, cq, path), node_(node)
{  
  current_joint_subscription_ =
    node_->create_subscription<tachimawari_interfaces::msg::CurrentJoints>(
      "/joint/current_joints", 10,
      [this](const tachimawari_interfaces::msg::CurrentJoints::SharedPtr curr_joints) {
        RCLCPP_INFO(rclcpp::get_logger("SubscribeCurrentJoints"), "current joints received from tachimawari");        
          curr_joints_.joints = curr_joints->joints;
      });
  Proceed();
}  // namespace akushon

void ConfigGrpc::CallDataSubscribeCurrentJoints::AddNextToCompletionQueue()
{
  new ConfigGrpc::CallDataSubscribeCurrentJoints(service_, cq_, path_, node_);
}

void ConfigGrpc::CallDataSubscribeCurrentJoints::WaitForRequest()
{
  service_->RequestSubscribeCurrentJoints(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void ConfigGrpc::CallDataSubscribeCurrentJoints::HandleRequest()
{
  try {
    
    nlohmann::json curr_joints;

    // curr_joints["control_type"] = curr_joints_->joints;
    for (const auto & items :  curr_joints_.joints) {
      nlohmann::json obj;
      obj["id"] = std::to_string(items.id);
      obj["position"] = std::to_string(items.position);
      curr_joints.push_back(obj);
    }

    reply_.set_msg_joints(curr_joints.dump());

    RCLCPP_INFO(rclcpp::get_logger("PublishSetTorques"), "curr joints has been sended!");

  } catch (std::ofstream::failure f) {
    RCLCPP_ERROR(rclcpp::get_logger("PublishSetTorques"), f.what());

  } catch (nlohmann::json::exception e) {
    RCLCPP_ERROR(rclcpp::get_logger("PublishSetTorques"), e.what());
  }
}

}  // namespace akushon
