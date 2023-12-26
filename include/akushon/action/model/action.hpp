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

#ifndef AKUSHON__ACTION__MODEL__ACTION_HPP_
#define AKUSHON__ACTION__MODEL__ACTION_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "akushon/action/model/pose.hpp"
#include "keisan/spline/smooth_spline.hpp"

namespace akushon
{

class Action
{
public:
  explicit Action(const std::string & action_name);

  void add_pose(const Pose & pose);
  void set_pose(int index, const Pose & pose);
  void delete_pose(int index);
  const Pose & get_pose(int index) const;
  const std::vector<Pose> & get_poses() const;

  void set_name(const std::string & action_name);
  const std::string & get_name() const;

  int get_pose_count() const;

  void set_start_delay(int start_delay);
  int get_start_delay() const;

  void set_stop_delay(int stop_delay);
  int get_stop_delay() const;

  void set_next_action(const std::string & next_action);
  const std::string & get_next_action() const;

  void reset();

  void enable_spline(bool enable);
  bool is_using_spline() const;
  void generate_splines();

  std::map<uint8_t, keisan::SmoothSpline *> joint_splines;

private:
  std::string name;

  std::vector<Pose> poses;

  bool use_spline;

  int stop_delay;
  int start_delay;

  std::string next_action;
};

}  // namespace akushon

#endif  // AKUSHON__ACTION__MODEL__ACTION_HPP_
