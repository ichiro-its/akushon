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

#ifndef AKUSHON__CONFIG__GRPC__CONFIG_HPP_
#define AKUSHON__CONFIG__GRPC__CONFIG_HPP_

#include <memory>
#include <thread>

#include "akushon_interfaces/akushon.grpc.pb.h"
#include "akushon_interfaces/akushon.pb.h"
#include "grpcpp/grpcpp.h"
#include "rclcpp/rclcpp.hpp"

using akushon_interfaces::proto::Config;

namespace akushon
{
class ConfigGrpc
{
public:
  explicit ConfigGrpc();
  explicit ConfigGrpc(const std::string & path);

  ~ConfigGrpc();

  void Run(uint16_t port, const std::string & path, rclcpp::Node::SharedPtr & node);

private:
  std::string path;

  static inline std::unique_ptr<grpc::ServerCompletionQueue> cq_;
  static inline std::unique_ptr<grpc::Server> server_;
  std::shared_ptr<std::thread> thread_;
  akushon_interfaces::proto::Config::AsyncService service_;

  std::thread async_server;
};

}  // namespace akushon

#endif  // AKUSHON__CONFIG__GRPC__CONFIG_HPP_
