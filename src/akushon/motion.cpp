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

#include <akushon/motion.hpp>

#include <string>

namespace akushon
{

Motion::Motion(std::string pose_name)
: name(pose_name)
{
}

void Motion::insert_pose(Pose pose)
{
  poses.push_back(pose);
}

void Motion::insert_pose(uint8_t id, Pose pose)
{
  poses.insert(poses.begin() + id, pose);
}

void Motion::delete_pose(uint8_t id)
{
  poses.erase(poses.begin() + id);
}

void Motion::set_name(std::string new_name)
{
  name = new_name;
}

std::string Motion::get_name()
{
  return name;
}

}  // namespace akushon
