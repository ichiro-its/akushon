// Copyright (c) 2021 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef AKUSHON__ACTION__MODEL__ACTION_NAME_HPP_
#define AKUSHON__ACTION__MODEL__ACTION_NAME_HPP_

#include <map>
#include <string>

namespace akushon
{

class ActionName
{
public:
  static const std::string INIT = "init";
  static const std::string WALKREADY = "walk_ready";
  static const std::string SIT_DOWN = "sit_down";
  static const std::string FORWARD_UP = "forward_up";
  static const std::string BACKWARD_UP = "backward_up";
  static const std::string LEFTWARD_UP = "leftward_ups";
  static const std::string RIGHTWARD_UP = "rightward_up";
  static const std::string RIGHT_KICK = "right_kick";
  static const std::string LEFT_KICK = "left_kick";
  static const std::string RIGHT_KICK_SHORT = "right_kick_short";
  static const std::string LEGT_KICK_SHORT = "left_kick_short";
  static const std::string LEFT_SIDEKICK = "left_sidekick";
  static const std::string RIGHT_SIDEKICK = "right_sidekick";
  static const std::string KEEPER_SIT = "keeper_sit";
  static const std::string KEEPER_UP = "keeper_up";

  static const std::map<std::string, int> map;
};

}  // namespace akushon

#endif  // AKUSHON__ACTION__MODEL__ACTION_NAME_HPP_
