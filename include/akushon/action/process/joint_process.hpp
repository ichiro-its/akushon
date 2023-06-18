// Copyright (c) 2021-2023 Ichiro ITS
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

#ifndef AKUSHON__ACTION__PROCESS__JOINT_PROCESS_HPP_
#define AKUSHON__ACTION__PROCESS__JOINT_PROCESS_HPP_

#include <string>
#include <vector>

#include "tachimawari/joint/model/joint.hpp"

namespace akushon
{

class JointProcess
{
public:
  explicit JointProcess(uint8_t joint_id, float position = 0.0);

  void set_target_position(float target_position, float speed = 1.0);
  void set_initial_position(float initial_position);

  void interpolate();

  bool is_finished() const;

  operator tachimawari::joint::Joint() const;

private:
  tachimawari::joint::Joint joint;

  float target_position;
  float initial_position;
  float additional_position;
};

}  // namespace akushon

#endif  // AKUSHON__ACTION__PROCESS__JOINT_PROCESS_HPP_
