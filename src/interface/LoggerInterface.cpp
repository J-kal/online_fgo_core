//  Copyright 2024 Institute of Automatic Control RWTH Aachen University
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//
//  Author: Haoming Zhang (h.zhang@irt.rwth-aachen.de)
//

#include "online_fgo_core/interface/LoggerInterface.h"
#include <iostream>
#include <chrono>
#include <iomanip>

namespace fgo::core {

  void ConsoleLogger::info(const std::string& message) {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::cout << "[INFO] [" << std::put_time(std::localtime(&time), "%H:%M:%S") 
              << "] " << message << std::endl;
  }

  void ConsoleLogger::warn(const std::string& message) {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::cout << "[WARN] [" << std::put_time(std::localtime(&time), "%H:%M:%S") 
              << "] " << message << std::endl;
  }

  void ConsoleLogger::error(const std::string& message) {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::cerr << "[ERROR] [" << std::put_time(std::localtime(&time), "%H:%M:%S") 
              << "] " << message << std::endl;
  }

  void ConsoleLogger::debug(const std::string& message) {
    if (debug_enabled_) {
      auto now = std::chrono::system_clock::now();
      auto time = std::chrono::system_clock::to_time_t(now);
      std::cout << "[DEBUG] [" << std::put_time(std::localtime(&time), "%H:%M:%S") 
                << "] " << message << std::endl;
    }
  }

} // namespace fgo::core
