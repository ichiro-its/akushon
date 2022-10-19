// Copyright (c) 2022 Ichiro ITS
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

#include <memory>

#include "gtest/gtest.h"

#include "akushon/action/model/action.hpp"
#include "akushon/action/model/pose.hpp"

TEST(PoseTest, ClassInitialization) {
  akushon::Pose pose("test pose");
  ASSERT_EQ(pose.get_name(), "test pose");
  ASSERT_EQ(action.get_speed(), 0.0);
  ASSERT_EQ(action.get_pause(), 0.0);
  ASSERT_TRUE(action.get_joints().empty());
}

TEST(PoseTest, SetterTest) {
  akushon::Pose pose("test pose");
  pose.set_name("new test pose");
  pose.set_speed(3.0);
  pose.set_pause(1.0);
  ASSERT_EQ(pose.get_name(), "new test pose");
  ASSERT_EQ(pose.get_speed(), 3.0);
  ASSERT_EQ(pose.get_pause(), 1.0);
}
