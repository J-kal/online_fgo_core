//  Copyright 2021 Institute of Automatic Control RWTH Aachen University
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
//  ROS-agnostic port for online_fgo_core
//

#ifndef ONLINE_FGO_CORE_FGO_PARAMETER_H
#define ONLINE_FGO_CORE_FGO_PARAMETER_H

#pragma once

#include "online_fgo_core/interface/ParameterInterface.h"
#include "online_fgo_core/interface/ApplicationInterface.h"
#include <utility>
#include <stdexcept>

namespace fgo::utils {

  /**
   * @brief Framework-agnostic parameter wrapper
   * 
   * Provides a similar interface to RosParameter but uses ParameterInterface.
   * This allows the same code pattern to work across ROS1, ROS2, and standalone applications.
   * 
   * Usage:
   *   FGOParameter<double> smootherLag("GNSSFGO.Optimizer.smootherLag", 0.1, app);
   *   double lag = smootherLag.value();
   */
  template <typename T>
  class FGOParameter {
  public:
    /**
     * @brief Constructor with default value
     * @param name Parameter name (supports nested names with dots)
     * @param default_value Default value if parameter not found
     * @param app Application interface for parameter access
     */
    FGOParameter(std::string name, T default_value, fgo::core::ApplicationInterface& app)
      : name_(std::move(name)), params_(app.getParameters()) {
      value_ = getParameterValue(name_, default_value);
    }

    /**
     * @brief Constructor with default value (pointer version)
     * @param name Parameter name
     * @param default_value Default value if parameter not found
     * @param app Application interface pointer
     */
    FGOParameter(std::string name, T default_value, fgo::core::ApplicationInterface* app)
      : name_(std::move(name)), params_(app->getParameters()) {
      if (!app) {
        throw std::invalid_argument("FGOParameter: ApplicationInterface pointer cannot be null");
      }
      value_ = getParameterValue(name_, default_value);
    }

    /**
     * @brief Constructor without default value (will throw if not found)
     * @param name Parameter name
     * @param app Application interface
     */
    FGOParameter(std::string name, fgo::core::ApplicationInterface& app)
      : name_(std::move(name)), params_(app.getParameters()) {
      if (!params_.hasParameter(name_)) {
        throw std::runtime_error("FGOParameter: Required parameter '" + name_ + "' not found");
      }
      value_ = getParameterValue(name_);
    }

    /**
     * @brief Constructor without default value (pointer version)
     */
    FGOParameter(std::string name, fgo::core::ApplicationInterface* app)
      : name_(std::move(name)), params_(app->getParameters()) {
      if (!app) {
        throw std::invalid_argument("FGOParameter: ApplicationInterface pointer cannot be null");
      }
      if (!params_.hasParameter(name_)) {
        throw std::runtime_error("FGOParameter: Required parameter '" + name_ + "' not found");
      }
      value_ = getParameterValue(name_);
    }

    /**
     * @brief Get the parameter value
     * @return Const reference to the parameter value
     */
    const T& value() const {
      return value_;
    }

    /**
     * @brief Get the parameter name
     * @return Parameter name
     */
    const std::string& name() const {
      return name_;
    }

    /**
     * @brief Refresh the parameter value from the parameter server
     * Useful if parameters can be changed at runtime
     * @return New value
     */
    const T& refresh() {
      value_ = getParameterValue(name_, value_);
      return value_;
    }

  private:
    std::string name_;
    fgo::core::ParameterInterface& params_;
    T value_;

    // Helper methods to call the correct type-specific getters
    template<typename U = T>
    typename std::enable_if<std::is_same<U, bool>::value, bool>::type
    getParameterValue(const std::string& name, bool default_val) {
      return params_.getBool(name, default_val);
    }

    template<typename U = T>
    typename std::enable_if<std::is_same<U, int>::value, int>::type
    getParameterValue(const std::string& name, int default_val) {
      return params_.getInt(name, default_val);
    }

    template<typename U = T>
    typename std::enable_if<std::is_same<U, double>::value, double>::type
    getParameterValue(const std::string& name, double default_val) {
      return params_.getDouble(name, default_val);
    }

    template<typename U = T>
    typename std::enable_if<std::is_same<U, std::string>::value, std::string>::type
    getParameterValue(const std::string& name, const std::string& default_val) {
      return params_.getString(name, default_val);
    }

    template<typename U = T>
    typename std::enable_if<std::is_same<U, std::vector<double>>::value, std::vector<double>>::type
    getParameterValue(const std::string& name, const std::vector<double>& default_val) {
      return params_.getDoubleArray(name, default_val);
    }

    template<typename U = T>
    typename std::enable_if<std::is_same<U, std::vector<int>>::value, std::vector<int>>::type
    getParameterValue(const std::string& name, const std::vector<int>& default_val) {
      return params_.getIntArray(name, default_val);
    }

    template<typename U = T>
    typename std::enable_if<std::is_same<U, std::vector<std::string>>::value, std::vector<std::string>>::type
    getParameterValue(const std::string& name, const std::vector<std::string>& default_val) {
      return params_.getStringArray(name, default_val);
    }

    // Versions without default value (for required parameters)
    template<typename U = T>
    typename std::enable_if<std::is_same<U, bool>::value, bool>::type
    getParameterValue(const std::string& name) {
      return params_.getBool(name, false);  // This will be checked by hasParameter before calling
    }

    template<typename U = T>
    typename std::enable_if<std::is_same<U, int>::value, int>::type
    getParameterValue(const std::string& name) {
      return params_.getInt(name, 0);
    }

    template<typename U = T>
    typename std::enable_if<std::is_same<U, double>::value, double>::type
    getParameterValue(const std::string& name) {
      return params_.getDouble(name, 0.0);
    }

    template<typename U = T>
    typename std::enable_if<std::is_same<U, std::string>::value, std::string>::type
    getParameterValue(const std::string& name) {
      return params_.getString(name, "");
    }

    template<typename U = T>
    typename std::enable_if<std::is_same<U, std::vector<double>>::value, std::vector<double>>::type
    getParameterValue(const std::string& name) {
      return params_.getDoubleArray(name, std::vector<double>());
    }

    template<typename U = T>
    typename std::enable_if<std::is_same<U, std::vector<int>>::value, std::vector<int>>::type
    getParameterValue(const std::string& name) {
      return params_.getIntArray(name, std::vector<int>());
    }

    template<typename U = T>
    typename std::enable_if<std::is_same<U, std::vector<std::string>>::value, std::vector<std::string>>::type
    getParameterValue(const std::string& name) {
      return params_.getStringArray(name, std::vector<std::string>());
    }
  };

  // Backwards compatibility alias (if needed)
  // This allows existing code using "RosParameter" to work with minimal changes
  template <typename T>
  using RosParameter = FGOParameter<T>;

} // namespace fgo::utils

// For compatibility with old code in the global utils namespace
namespace utils {
  template <typename T>
  using RosParameter = fgo::utils::FGOParameter<T>;
}

#endif // ONLINE_FGO_CORE_FGO_PARAMETER_H
