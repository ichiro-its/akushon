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

#ifndef AKUSHON__ACTION__MODEL__ACTION_HPP_
#define AKUSHON__ACTION__MODEL__ACTION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "akushon/action/model/pose.hpp"

namespace akushon
{

class Action
{
public:
  enum
  {
    INIT,
    WALKREADY,
    SIT_DOWN,
    FORWARD_UP,
    BACKWARD_UP,
    LEFTWARD_UP,
    RIGHTWARD_UP,
    RIGHT_KICK,
    LEFT_KICK,
    RIGHT_KICK_SHORT,
    LEGT_KICK_SHORT,
    LEFT_SIDEKICK,
    RIGHT_SIDEKICK,
    KEEPER_SIT,
    KEEPER_UP,
    TOTAL
  };

  explicit Action(const std::string & action_name);

  void add_pose(const Pose & pose);
  void set_pose(const int & index, const Pose & pose);
  void delete_pose(const int & index);
  const Pose & get_pose(const int & index) const;
  const std::vector<Pose> & get_poses() const;

  void set_name(const std::string & action_name);
  const std::string & get_name() const;

  const int & get_pose_count() const;

  void reset();

private:
  std::string name;

  std::vector<Pose> poses;

  int stop_delay;
  int start_delay;
};

}  // namespace akushon

#endif  // AKUSHON__ACTION__MODEL__ACTION_HPP_
