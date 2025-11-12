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

#ifndef ONLINE_FGO_CORE_LOGGER_INTERFACE_H
#define ONLINE_FGO_CORE_LOGGER_INTERFACE_H

#pragma once

#include <string>
#include <sstream>
#include <memory>

namespace fgo::core {

  /**
   * @brief Abstract interface for logging functionality
   * 
   * This interface allows the core FGO library to perform logging
   * without depending on any specific logging framework (ROS, spdlog, etc.)
   */
  class LoggerInterface {
  public:
    virtual ~LoggerInterface() = default;

    /**
     * @brief Log an informational message
     * @param message The message to log
     */
    virtual void info(const std::string& message) = 0;

    /**
     * @brief Log a warning message
     * @param message The message to log
     */
    virtual void warn(const std::string& message) = 0;

    /**
     * @brief Log an error message
     * @param message The message to log
     */
    virtual void error(const std::string& message) = 0;

    /**
     * @brief Log a debug message
     * @param message The message to log
     */
    virtual void debug(const std::string& message) = 0;

    /**
     * @brief Check if debug logging is enabled
     * @return true if debug messages will be logged
     */
    virtual bool isDebugEnabled() const { return false; }

    // Stream-style logging helpers
    template<typename... Args>
    void infoStream(Args&&... args) {
      std::ostringstream oss;
      (oss << ... << args);
      info(oss.str());
    }

    template<typename... Args>
    void warnStream(Args&&... args) {
      std::ostringstream oss;
      (oss << ... << args);
      warn(oss.str());
    }

    template<typename... Args>
    void errorStream(Args&&... args) {
      std::ostringstream oss;
      (oss << ... << args);
      error(oss.str());
    }

    template<typename... Args>
    void debugStream(Args&&... args) {
      if (isDebugEnabled()) {
        std::ostringstream oss;
        (oss << ... << args);
        debug(oss.str());
      }
    }
  };

  using LoggerPtr = std::shared_ptr<LoggerInterface>;

  /**
   * @brief Default console logger implementation
   */
  class ConsoleLogger : public LoggerInterface {
  public:
    void info(const std::string& message) override;
    void warn(const std::string& message) override;
    void error(const std::string& message) override;
    void debug(const std::string& message) override;
    bool isDebugEnabled() const override { return debug_enabled_; }
    
    void setDebugEnabled(bool enabled) { debug_enabled_ = enabled; }

  private:
    bool debug_enabled_ = false;
  };

} // namespace fgo::core

#endif // ONLINE_FGO_CORE_LOGGER_INTERFACE_H
