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

#ifndef AKUSHON__ACTION__MODEL__POSE_HPP_
#define AKUSHON__ACTION__MODEL__POSE_HPP_

#include <string>
#include <vector>

#include "tachimawari/joint/model/joint.hpp"

namespace akushon
{

class Pose
{
public:
  explicit Pose(const std::string & pose_name);

  void set_speed(const float & speed);
  const float & get_speed() const;

  void set_pause(const float & pause);
  const float & get_pause() const;

  void set_name(const std::string & pose_name);
  const std::string & get_name() const;

  void set_joints(const std::vector<tachimawari::joint::Joint> & joints);
  const std::vector<tachimawari::joint::Joint> & get_joints() const;

  void set_target_position(const Pose & target_pose);

private:
  float speed;
  float pause;

  std::string name;

  std::vector<tachimawari::joint::Joint> joints;
};

}  // namespace akushon

#endif  // AKUSHON__ACTION__MODEL__POSE_HPP_
