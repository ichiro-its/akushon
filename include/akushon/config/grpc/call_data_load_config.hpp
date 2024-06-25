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

#ifndef AKUSHON__CONFIG__GRPC__LOAD_CONFIG_HPP_
#define AKUSHON__CONFIG__GRPC__LOAD_CONFIG_HPP_

#include "akushon/action/node/action_manager.hpp"
#include "akushon/config/grpc/call_data.hpp"

namespace akushon
{
class CallDataLoadConfig
: CallData<akushon_interfaces::proto::Empty, akushon_interfaces::proto::Empty>
{
public:
  CallDataLoadConfig(
    akushon_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
    const std::string& path, const std::shared_ptr<ActionManager>& action_manager);

protected:
  void AddNextToCompletionQueue() override;
  void WaitForRequest() override;
  void HandleRequest() override;
  std::shared_ptr<ActionManager> action_manager_;
};
}  // namespace akushon

#endif // AKUSHON__CONFIG__GRPC__LOAD_CONFIG_HPP_
