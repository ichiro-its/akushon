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

#ifndef AKUSHON__ACTION__PROCESS__INTERPOLATOR_HPP_
#define AKUSHON__ACTION__PROCESS__INTERPOLATOR_HPP_

#include <map>
#include <string>
#include <vector>

#include "akushon/action/model/action.hpp"
#include "akushon/action/model/pose.hpp"
#include "akushon/action/process/joint_process.hpp"

namespace akushon
{

class Interpolator
{
public:
  enum
  {
    START_DELAY,
    PLAYING,
    STOP_DELAY,
    END
  };

  explicit Interpolator(const std::vector<Action> & actions, const Pose & initial_pose);

  void process(int time);
  bool is_finished() const;

  std::vector<tachimawari::joint::Joint> get_joints() const;

private:
  const Action & get_current_action() const;
  const Pose & get_current_pose() const;

  bool check_for_next();
  void next_pose();

  void change_state(int state);

  std::vector<Action> actions;

  int state;
  bool init_state;

  int start_stop_time;
  bool init_pause;
  int pause_time;

  int current_action_index;
  int current_pose_index;

  std::map<uint8_t, JointProcess> joint_processes;
};

}  // namespace akushon

#endif  // AKUSHON__ACTION__PROCESS__INTERPOLATOR_HPP_
