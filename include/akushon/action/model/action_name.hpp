// Copyright (c) 2021-2023 Ichiro ITS
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
  static const char * INIT;
  static const char * WALKREADY;
  static const char * SIT_DOWN;
  static const char * FORWARD_UP;
  static const char * BACKWARD_UP;
  static const char * LEFTWARD_UP;
  static const char * RIGHTWARD_UP;
  static const char * RIGHT_KICK;
  static const char * LEFT_KICK;
  static const char * RIGHT_KICK_SHORT;
  static const char * LEFT_KICK_SHORT;
  static const char * RIGHT_KICK_WIDE;
  static const char * LEFT_KICK_WIDE;
  static const char * LEFT_SIDEKICK;
  static const char * RIGHT_SIDEKICK;
  static const char * KEEPER_SIT;
  static const char * KEEPER_UP;

  static const std::map<std::string, int> map;
};

}  // namespace akushon

#endif  // AKUSHON__ACTION__MODEL__ACTION_NAME_HPP_
