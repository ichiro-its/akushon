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

#ifndef AKUSHON__ACTION_HPP_
#define AKUSHON__ACTION_HPP_

#include <akushon/pose.hpp>
#include <nlohmann/json.hpp>

#include <memory>
#include <string>
#include <vector>

namespace akushon
{

class Action
{
public:
  explicit Action(const std::string & action_name);

  void insert_pose(const Pose & pose);
  void insert_pose(const uint8_t & id, const Pose & pose);
  void delete_pose(const uint8_t & id);

  void load_data(const std::string & path);
  void set_name(const std::string & action_name);

  const std::string & get_name() const;
  const std::vector<Pose> & get_poses() const;
  const Pose & get_current_pose() const;
  const Pose & get_pose_by_index(const uint8_t & index) const;

  void next_pose();
  bool is_running() const;

  Pose process(Pose robot_pose, const int & time);
  void reset();

private:
  std::string name;

  int current_pose_index = 0;
  int pose_count = 0;
  uint8_t next_motion_id;

  std::vector<Pose> poses;

  int pause_start_time;
  bool on_pause;
  bool on_process;
};

}  // namespace akushon

#endif  // AKUSHON__ACTION_HPP_
