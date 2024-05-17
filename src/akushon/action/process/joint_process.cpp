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

#include <string>
#include <vector>
#include <cmath>

#include "akushon/action/process/joint_process.hpp"

#include "tachimawari/joint/model/joint.hpp"

namespace akushon
{

JointProcess::JointProcess(uint8_t joint_id, float position)
: joint(tachimawari::joint::Joint(joint_id, position)), initial_position(position),
  target_position(position), additional_position(0.0), initial_time(0.0), action_time(0.0)
{
}

void JointProcess::set_target_position_time(float target_position, double time, float action_time)
{
  this->target_position = target_position;
  this->action_time = action_time;
  this->initial_time = time;
}

void JointProcess::set_target_position(float target_position, float speed)
{
  float filtered_speed = (speed > 1.0) ? 1.0 : speed;
  filtered_speed = (filtered_speed < 0.0) ? 0.0 : filtered_speed;

  this->target_position = target_position;
  additional_position = (this->target_position - initial_position) * filtered_speed;
  additional_position = (fabs(additional_position) < 0.1) ? 0.0 : additional_position;
}

void JointProcess::set_initial_position(float initial_position)
{
  this->initial_position = initial_position;
}

void JointProcess::interpolate_time(double time)
{
  bool time_is_reached = false;
  double passed_time = time - initial_time;
  double divider = passed_time / action_time;

  printf("passed_time: %1.f", passed_time);
  printf("action_time: %1.f", action_time);

  time_is_reached |= (divider >= 1.0);

  if (time_is_reached) {
      joint.set_position(target_position);
      additional_position = 0.0;
      initial_position = target_position;
  } else {
      additional_position = (target_position - initial_position) * divider;
      additional_position = (fabs(additional_position) < 0.1) ? 0.0 : additional_position;
      joint.set_position(initial_position + additional_position);
  }
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
  return (initial_position == target_position) || (additional_position == 0.0);
}

JointProcess::operator tachimawari::joint::Joint() const
{
  return joint;
}

}  // namespace akushon
