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

#include <akushon/pose.hpp>

#include <string>
#include <vector>

namespace akushon
{

Pose::Pose(std::string pose_name)
: name(pose_name)
{
}

void Pose::set_speed(float speed)
{
  this->speed = speed;
}

float Pose::get_speed()
{
  return speed;
}

void Pose::set_pause(float pause)
{
  this->pause = pause;
}

float Pose::get_pause()
{
  return pause;
}

void Pose::set_name(std::string pose_name)
{
  name = pose_name;
}

std::string Pose::get_name()
{
  return name;
}

void Pose::interpolate() 
{
  for (auto & current_joint: this->joints) {
    while(current_joint.get_position()!= current_joint.get_goal_position()) {
      current_joint.interpolate();
    }
  }
}

void Pose::set_joints(std::vector<tachimawari::Joint> joints)
{
  this->joints = joints;
}

std::vector<tachimawari::Joint> Pose::get_joints()
{
  return joints;
}

bool Pose::is_equals(Pose other_pose) {
  std::vector<tachimawari::Joint> joints = other_pose.get_joints();
  
  for (auto & joint: this->joints) {
    for (auto & other_joint: other_pose.get_joints()) {
      if(joint.get_joint_name() == other_joint.get_joint_name()) {
        if(joint.get_position() != other_joint.get_goal_position()) {
          return false;
        }
      }
    }
  }
  
  return true;
}

}  // namespace akushon
