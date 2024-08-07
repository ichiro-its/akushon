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
#include "akushon/config/grpc/call_data_brake_action.hpp"
#include "akushon/config/grpc/call_data_get_config.hpp"
#include "akushon/config/grpc/call_data_load_config.hpp"
#include "akushon/config/grpc/call_data_save_config.hpp"
#include "akushon/config/grpc/call_data_publish_set_joints.hpp"
#include "akushon/config/grpc/call_data_publish_set_torques.hpp"
#include "akushon/config/grpc/call_data_subscribe_current_joints.hpp"
#include "akushon/config/grpc/call_data_run_action.hpp"

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

void ConfigGrpc::Run(uint16_t port, const std::string& path, rclcpp::Node::SharedPtr& node,
  const std::shared_ptr<ActionManager>& action_manager)
{
  std::string server_address = absl::StrFormat("0.0.0.0:%d", port);

  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service_);

  cq_ = builder.AddCompletionQueue();
  server_ = builder.BuildAndStart();
  std::cout << "Server listening on " << server_address << std::endl;

  std::signal(SIGINT, [](int signum) {
    server_->Shutdown();
    cq_->Shutdown();
    exit(signum);
  });
  async_server = std::thread([path, this, &node, &action_manager]() {
    new CallDataBrakeAction(&service_, cq_.get(), path, action_manager);
    new CallDataGetConfig(&service_, cq_.get(), path);
    new CallDataSaveConfig(&service_, cq_.get(), path, action_manager);
    new CallDataPublishSetJoints(&service_, cq_.get(), path, node);
    new CallDataPublishSetTorques(&service_, cq_.get(), path, node);
    new CallDataRunAction(&service_, cq_.get(), path, node);
    new CallDataSubscribeCurrentJoints(&service_, cq_.get(), path, node);
    new CallDataLoadConfig(&service_, cq_.get(), path, action_manager);
    void * tag;  // uniquely identifies a request.
    bool ok = true;
    while (true) {
      this->cq_->Next(&tag, &ok);
      if (ok) {
        static_cast<CallDataBase *>(tag)->Proceed();
      }
    }
  });
  std::this_thread::sleep_for(200ms);
}

}  // namespace akushon
