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

#ifndef ONLINE_FGO_CORE_APPLICATION_INTERFACE_H
#define ONLINE_FGO_CORE_APPLICATION_INTERFACE_H

#pragma once

#include <memory>
#include <string>

#include "LoggerInterface.h"
#include "TimeInterface.h"
#include "ParameterInterface.h"
#include "PublisherInterface.h"

namespace fgo::core {

  /**
   * @brief Main application interface for the FGO core library
   * 
   * This interface provides access to all framework-specific functionality
   * (logging, time, parameters, publishing) through a single interface.
   * This replaces the direct dependency on rclcpp::Node or ros::NodeHandle.
   */
  class ApplicationInterface {
  public:
    virtual ~ApplicationInterface() = default;

    /**
     * @brief Get the logger interface
     * @return Reference to logger
     */
    virtual LoggerInterface& getLogger() = 0;

    /**
     * @brief Get the parameter interface
     * @return Reference to parameter server
     */
    virtual ParameterInterface& getParameters() = 0;

    /**
     * @brief Get the current time
     * @return Current timestamp
     */
    virtual TimeStamp now() const = 0;

    /**
     * @brief Create a publisher for a specific message type
     * @tparam MsgType The message type
     * @param topic The topic name
     * @return Shared pointer to publisher
     */
    template<typename MsgType>
    std::shared_ptr<PublisherInterface<MsgType>> createPublisher(const std::string& topic) {
      return createPublisherImpl<MsgType>(topic);
    }

    /**
     * @brief Get application name/node name
     * @return Name of the application
     */
    virtual std::string getName() const = 0;

    /**
     * @brief Check if the application is still running
     * @return true if running, false if shutdown requested
     */
    virtual bool ok() const { return true; }

  protected:
    /**
     * @brief Implementation-specific publisher creation
     * Must be implemented by derived classes
     */
    template<typename MsgType>
    std::shared_ptr<PublisherInterface<MsgType>> createPublisherImpl(const std::string& topic);
  };

  using ApplicationPtr = std::shared_ptr<ApplicationInterface>;

  /**
   * @brief Standalone application implementation
   * 
   * Can be used without ROS for testing or standalone applications.
   */
  class StandaloneApplication : public ApplicationInterface {
  public:
    explicit StandaloneApplication(const std::string& name = "online_fgo")
      : name_(name),
        logger_(std::make_shared<ConsoleLogger>()),
        params_(std::make_shared<MapParameterServer>()) {}

    LoggerInterface& getLogger() override { return *logger_; }
    ParameterInterface& getParameters() override { return *params_; }
    TimeStamp now() const override { return TimeStamp::now(); }
    std::string getName() const override { return name_; }

    // Access to concrete implementations for configuration
    std::shared_ptr<ConsoleLogger> getConsoleLogger() { return logger_; }
    std::shared_ptr<MapParameterServer> getParameterServer() { return params_; }

    template<typename MsgType>
    std::shared_ptr<PublisherInterface<MsgType>> createPublisher(const std::string& /*topic*/) {
      // For standalone, return null publishers
      return std::make_shared<NullPublisher<MsgType>>();
    }

  private:
    std::string name_;
    std::shared_ptr<ConsoleLogger> logger_;
    std::shared_ptr<MapParameterServer> params_;
  };

} // namespace fgo::core

#endif // ONLINE_FGO_CORE_APPLICATION_INTERFACE_H
