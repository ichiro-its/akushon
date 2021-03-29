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

#ifndef AKUSHON__POSE_HPP_
#define AKUSHON__POSE_HPP_

#include <tachimawari/joint.hpp>

#include <string>
#include <vector>

namespace akushon
{

class Pose
{
public:
  explicit Pose(std::string pose_name);

  void set_velocity(float new_velocity);
  float get_velocity();

  void set_pause(float new_pause);
  float get_pause();

  void set_name(std::string new_name);
  std::string get_name();

  void set_joints(std::vector<tachimawari::Joint> joints);
  std::vector<tachimawari::Joint> get_joints();

private:
  float velocity;
  float pause;

  std::string name;

  static std::vector<tachimawari::Joint> joints;
};

}  // namespace akushon

#endif  // AKUSHON__POSE_HPP_
