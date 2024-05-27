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

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "akushon/action/model/action.hpp"

#include "akushon/action/model/pose.hpp"

namespace akushon
{

Action::Action(const std::string & action_name)
: name(action_name), poses({}), start_delay(0), stop_delay(0), time_based(false), next_action("")
{
}

void Action::add_pose(const Pose & pose)
{
  poses.push_back(pose);
}

void Action::set_pose(int index, const Pose & pose)
{
  poses.insert(poses.begin() + index, pose);
}

void Action::delete_pose(int index)
{
  poses.erase(poses.begin() + index);
}

void Action::set_name(const std::string & action_name)
{
  name = action_name;
}

const std::string & Action::get_name() const
{
  return name;
}

const std::vector<Pose> & Action::get_poses() const
{
  return poses;
}

const Pose & Action::get_pose(int index) const
{
  return poses.at(index);
}

int Action::get_pose_count() const
{
  return poses.size();
}

void Action::set_start_delay(int start_delay)
{
  this->start_delay = start_delay;
}

int Action::get_start_delay() const
{
  return start_delay;
}

void Action::set_stop_delay(int stop_delay)
{
  this->stop_delay = stop_delay;
}

int Action::get_stop_delay() const
{
  return stop_delay;
}

void Action::set_next_action(const std::string & next_action)
{
  this->next_action = next_action;
}

const std::string & Action::get_next_action() const
{
  return next_action;
}

void Action::reset()
{
  poses.clear();
}

}  // namespace akushon
