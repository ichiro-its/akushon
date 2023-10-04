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

#ifndef AKUSHON__CONFIG__GRPC__CONFIG_HPP_
#define AKUSHON__CONFIG__GRPC__CONFIG_HPP_

#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/strings/str_format.h"
#include "akushon.grpc.pb.h"
#include "akushon.pb.h"
#include "grpc/support/log.h"
#include "grpcpp/grpcpp.h"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tachimawari_interfaces/msg/set_joints.hpp"
#include "tachimawari_interfaces/msg/set_torques.hpp"
#include "tachimawari_interfaces/msg/current_joints.hpp"
#include "akushon_interfaces/msg/run_action.hpp"

using akushon_interfaces::proto::Config;

namespace akushon
{
class ConfigGrpc
{
public:
  explicit ConfigGrpc();
  explicit ConfigGrpc(const std::string & path);

  ~ConfigGrpc();

  void Run(uint16_t port, const std::string path, rclcpp::Node::SharedPtr node);

private:
  std::string path;
  static void SignIntHandler(int signum);

  class CallDataBase
  {
  public:
    CallDataBase();

    virtual void Proceed() = 0;

  protected:
    virtual void WaitForRequest() = 0;
    virtual void HandleRequest() = 0;
  };

  template <class ConfigRequest, class ConfigReply>
  class CallData : CallDataBase
  {
  public:
    CallData(
      akushon_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
      const std::string path);

    virtual void Proceed() override;

  protected:
    virtual void AddNextToCompletionQueue() = 0;

    enum CallStatus { CREATE, PROCESS, FINISH };

    CallStatus status_;  // The current serving state.

    akushon_interfaces::proto::Config::AsyncService * service_;

    const std::string path_;

    grpc::ServerCompletionQueue * cq_;
    grpc::ServerContext ctx_;
    ConfigRequest request_;
    ConfigReply reply_;
    grpc::ServerAsyncResponseWriter<ConfigReply> responder_;
  };

  class CallDataGetConfig
  : CallData<akushon_interfaces::proto::Empty, akushon_interfaces::proto::ConfigActions>
  {
  public:
    CallDataGetConfig(
      akushon_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
      const std::string path);

  protected:
    virtual void AddNextToCompletionQueue() override;
    virtual void WaitForRequest() override;
    virtual void HandleRequest() override;
  };

  class CallDataSaveConfig
  : CallData<akushon_interfaces::proto::ConfigActions, akushon_interfaces::proto::Empty>
  {
  public:
    CallDataSaveConfig(
      akushon_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
      const std::string path);

  protected:
    virtual void AddNextToCompletionQueue() override;
    virtual void WaitForRequest() override;
    virtual void HandleRequest() override;
  };

  class CallDataPublishSetJoints
  : CallData<akushon_interfaces::proto::SetJointsData, akushon_interfaces::proto::Empty>
  {
  public:
    CallDataPublishSetJoints(
      akushon_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
      const std::string path, rclcpp::Node::SharedPtr node);

  protected:
    virtual void AddNextToCompletionQueue() override;
    virtual void WaitForRequest() override;
    virtual void HandleRequest() override;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<tachimawari_interfaces::msg::SetJoints>::SharedPtr set_joints_publisher_;
  };

  class CallDataRunAction
  : CallData<akushon_interfaces::proto::ConfigRunAction, akushon_interfaces::proto::Empty>
  {
  public:
    CallDataRunAction(
      akushon_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
      const std::string path, rclcpp::Node::SharedPtr node);

  protected:
    virtual void AddNextToCompletionQueue() override;
    virtual void WaitForRequest() override;
    virtual void HandleRequest() override;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<akushon_interfaces::msg::RunAction>::SharedPtr run_action_publisher_;
  };

  class CallDataPublishSetTorques
  : CallData<akushon_interfaces::proto::SetTorquesData, akushon_interfaces::proto::Empty>
  {
  public:
    CallDataPublishSetTorques(
      akushon_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
      const std::string path, rclcpp::Node::SharedPtr node);

  protected:
    virtual void AddNextToCompletionQueue() override;
    virtual void WaitForRequest() override;
    virtual void HandleRequest() override;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<tachimawari_interfaces::msg::SetTorques>::SharedPtr set_torque_publisher_;
  };

  class CallDataSubscribeCurrentJoints
  : CallData<akushon_interfaces::proto::Empty, akushon_interfaces::proto::CurrentJoints>
  {
  public:
    CallDataSubscribeCurrentJoints(
      akushon_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
      const std::string path, rclcpp::Node::SharedPtr node);

  protected:
    virtual void AddNextToCompletionQueue() override;
    virtual void WaitForRequest() override;
    virtual void HandleRequest() override;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<tachimawari_interfaces::msg::CurrentJoints>::SharedPtr current_joint_subscription_;
    tachimawari_interfaces::msg::CurrentJoints curr_joints_;
  };

  static inline std::unique_ptr<grpc::ServerCompletionQueue> cq_;
  static inline std::unique_ptr<grpc::Server> server_;
  std::shared_ptr<std::thread> thread_;
  akushon_interfaces::proto::Config::AsyncService service_;

  std::thread async_server;
};

}  // namespace akushon

#endif  // AKUSHON__CONFIG__GRPC__CONFIG_HPP_
