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

#include <akushon/action.hpp>

#include <string>
#include <vector>

namespace akushon
{

Action::Action(std::string action_name)
: name(action_name), pose_index(0)
{
}

void Action::insert_pose(Pose pose)
{
  poses.push_back(pose);
  pose_count = poses.size();
}

void Action::insert_pose(uint8_t id, Pose pose)
{
  poses.insert(poses.begin() + id, pose);
  pose_count = poses.size();
}

void Action::delete_pose(uint8_t id)
{
  poses.erase(poses.begin() + id);
  pose_count = poses.size();
}

void Action::set_name(std::string action_name)
{
  name = action_name;
}

std::string Action::get_name()
{
  return name;
}

std::vector<Pose> Action::get_poses()
{
  return poses;
}

Pose Action::get_pose()
{
  return poses.at(pose_index);
}

Pose Action::get_pose_at_index(uint8_t id)
{
  return poses.at(id);
}

void Action::next_pose()
{
  pose_index++;
}

int Action::get_pose_index()
{
  return pose_index;
}

bool Action::is_finished()
{
  return pose_index == pose_count;
}

}  // namespace akushon
