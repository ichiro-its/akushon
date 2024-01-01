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

#include "akushon/config/grpc/call_data.hpp"

namespace akushon
{
template <class ConfigRequest, class ConfigReply>
CallData<ConfigRequest, ConfigReply>::CallData(
    akushon_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
    const std::string& path) : status_(CallStatus::CREATE), service_(service), cq_(cq), responder_(&ctx_), path_(path) {
}

template <class ConfigRequest, class ConfigReply>
void CallData<ConfigRequest, ConfigReply>::Proceed()
{
  if (status_ == CallStatus::CREATE) {
    status_ = CallStatus::PROCESS;
    WaitForRequest();
  } else if (status_ == CallStatus::PROCESS) {
    AddNextToCompletionQueue();
    HandleRequest();
    status_ = CallStatus::FINISH;
    responder_.Finish(reply_, grpc::Status::OK, this);
  } else {
    GPR_ASSERT(status_ == CallStatus::FINISH);
    delete this;
  }
}

template class CallData<akushon_interfaces::proto::ConfigActions, akushon_interfaces::proto::Empty>;
template class CallData<akushon_interfaces::proto::Empty, akushon_interfaces::proto::ConfigActions>;
template class CallData<akushon_interfaces::proto::SetJointsData, akushon_interfaces::proto::Empty>;
template class CallData<
  akushon_interfaces::proto::SetTorquesData, akushon_interfaces::proto::Empty>;
template class CallData<akushon_interfaces::proto::Empty, akushon_interfaces::proto::CurrentJoints>;
template class CallData<
  akushon_interfaces::proto::ConfigRunAction, akushon_interfaces::proto::Empty>;
}
