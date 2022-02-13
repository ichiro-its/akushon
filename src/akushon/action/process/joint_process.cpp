// Copyright (c) 2021 Ichiro ITS
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

#include <string>
#include <vector>

#include "akushon/action/process/joint_process.hpp"

#include "tachimawari/joint/model/joint.hpp"

namespace akushon
{

JointProcess::JointProcess(uint8_t joint_id, float position)
: joint(tachimawari::joint::Joint(joint_id, position)), initial_position(position),
  target_position(position), additional_position(0.0)
{
}

void JointProcess::set_target_position(float target_position, float speed)
{
  float filtered_speed = (speed > 1.0) ? 1.0 : speed;
  filtered_speed = (filtered_speed < 0.0) ? 0.0 : filtered_speed;

  this->target_position = target_position;
  additional_position = (this->target_position - initial_position) * filtered_speed;
}

void JointProcess::set_initial_position(float initial_position)
{
  this->initial_position = initial_position;
}

void JointProcess::interpolate()
{
  bool target_position_is_reached = false;
  target_position_is_reached |= (additional_position >= 0 &&
    (joint.get_position() + additional_position >= target_position));
  target_position_is_reached |= (additional_position <= 0 &&
    (joint.get_position() + additional_position < target_position));

  if (target_position_is_reached) {
    joint.set_position(target_position);
    additional_position = 0.0;
    initial_position = target_position;
  } else {
    joint.set_position(joint.get_position() + additional_position);
  }
}

bool JointProcess::is_finished() const
{
  return initial_position == target_position;
}

JointProcess::operator tachimawari::joint::Joint() const
{
  return joint;
}

}  // namespace akushon
