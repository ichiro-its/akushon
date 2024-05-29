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

#include "akushon/action/model/pose.hpp"

#include "tachimawari/joint/model/joint.hpp"

namespace akushon
{

Pose::Pose(const std::string & pose_name)
: name(pose_name), speed(0.0), pause(0.0), action_time(0.0), joints({})
{
}

void Pose::set_speed(float speed)
{
  this->speed = speed;
}

float Pose::get_speed() const
{
  return speed;
}

void Pose::set_pause(float pause)
{
  this->pause = pause;
}

float Pose::get_pause() const
{
  return pause;
}

void Pose::set_name(const std::string & pose_name)
{
  name = pose_name;
}

const std::string & Pose::get_name() const
{
  return name;
}

void Pose::set_joints(const std::vector<tachimawari::joint::Joint> & joints)
{
  this->joints = joints;
}

const std::vector<tachimawari::joint::Joint> & Pose::get_joints() const
{
  return joints;
}

}  // namespace akushon
