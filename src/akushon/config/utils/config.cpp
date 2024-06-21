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

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "akushon/action/model/action_name.hpp"
#include "akushon/config/utils/config.hpp"
#include "jitsuyo/config.hpp"
#include "nlohmann/json.hpp"

namespace akushon
{

Config::Config(const std::string & path)
: path(path)
{
}

std::string Config::get_config() const
{
  nlohmann::json actions_list;
  std::cout << "[ ACTIONS LIST ] : " << std::endl;
  for (const auto & action_file : std::filesystem::directory_iterator(path)) {
    std::string file_name = action_file.path();

    std::string action_name = "";
    for (auto i = path.size(); i < file_name.size() - 5; i++) {
      action_name += file_name[i];
    }
    std::cout << action_name << " | ";

    std::ifstream file(action_file.path());
    nlohmann::json action_data;
    if (!jitsuyo::load_config(path, action_name + ".json", action_data)) {
      std::cerr << "Failed to load " << file_name << std::endl;
      continue;
    }

    if (action_data["name"] != action_name) {
      std::cerr << "Action name does not match file name in " << file_name << std::endl;
      continue;
    }

    if (action_data["poses"].empty()) {
      std::cerr << file_name << "\'s poses is empty" << std::endl;
      continue;
    }

    actions_list[action_name] = action_data;
  }
  std::cout << std::endl;
  return actions_list.dump();
}

void Config::save_config(const std::string & actions_data)
{
  nlohmann::json actions_list = nlohmann::json::parse(actions_data);
  for (const auto & [key, val] : actions_list.items()) {
    std::locale loc;
    std::string action_name = key;
    std::replace(action_name.begin(), action_name.end(), ' ', '_');
    std::string file_name = path + action_name + ".json";
    std::ofstream file;

    if (!jitsuyo::save_config(path, action_name + ".json", val)) {
      std::cerr << "Failed to save " << file_name << std::endl;
      continue;
    }
  }
}

}  // namespace akushon
