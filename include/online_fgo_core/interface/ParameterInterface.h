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

#ifndef ONLINE_FGO_CORE_PARAMETER_INTERFACE_H
#define ONLINE_FGO_CORE_PARAMETER_INTERFACE_H

#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <stdexcept>
#include <sstream>

namespace fgo::core {

  /**
   * @brief Abstract interface for parameter management
   * 
   * This interface allows the core FGO library to access configuration
   * parameters without depending on ROS parameter server or other frameworks.
   */
  class ParameterInterface {
  public:
    virtual ~ParameterInterface() = default;

    /**
     * @brief Get a boolean parameter
     * @param name Parameter name (can use dot notation like "GNSSFGO.Graph.publishResiduals")
     * @param default_value Value to return if parameter doesn't exist
     * @return Parameter value or default
     */
    virtual bool getBool(const std::string& name, bool default_value) = 0;

    /**
     * @brief Get an integer parameter
     * @param name Parameter name
     * @param default_value Value to return if parameter doesn't exist
     * @return Parameter value or default
     */
    virtual int getInt(const std::string& name, int default_value) = 0;

    /**
     * @brief Get a double parameter
     * @param name Parameter name
     * @param default_value Value to return if parameter doesn't exist
     * @return Parameter value or default
     */
    virtual double getDouble(const std::string& name, double default_value) = 0;

    /**
     * @brief Get a string parameter
     * @param name Parameter name
     * @param default_value Value to return if parameter doesn't exist
     * @return Parameter value or default
     */
    virtual std::string getString(const std::string& name, const std::string& default_value) = 0;

    /**
     * @brief Get a vector of doubles
     * @param name Parameter name
     * @param default_value Value to return if parameter doesn't exist
     * @return Parameter value or default
     */
    virtual std::vector<double> getDoubleArray(const std::string& name, 
                                               const std::vector<double>& default_value) = 0;

    /**
     * @brief Get a vector of integers
     * @param name Parameter name
     * @param default_value Value to return if parameter doesn't exist
     * @return Parameter value or default
     */
    virtual std::vector<int> getIntArray(const std::string& name, 
                                         const std::vector<int>& default_value) = 0;

    /**
     * @brief Get a vector of strings
     * @param name Parameter name
     * @param default_value Value to return if parameter doesn't exist
     * @return Parameter value or default
     */
    virtual std::vector<std::string> getStringArray(const std::string& name, 
                                                    const std::vector<std::string>& default_value) = 0;

    /**
     * @brief Check if a parameter exists
     * @param name Parameter name
     * @return true if parameter exists
     */
    virtual bool hasParameter(const std::string& name) const = 0;

    /**
     * @brief Set a boolean parameter
     * @param name Parameter name
     * @param value Parameter value
     */
    virtual void setBool(const std::string& name, bool value) = 0;

    /**
     * @brief Set an integer parameter
     * @param name Parameter name
     * @param value Parameter value
     */
    virtual void setInt(const std::string& name, int value) = 0;

    /**
     * @brief Set a double parameter
     * @param name Parameter name
     * @param value Parameter value
     */
    virtual void setDouble(const std::string& name, double value) = 0;

    /**
     * @brief Set a string parameter
     * @param name Parameter name
     * @param value Parameter value
     */
    virtual void setString(const std::string& name, const std::string& value) = 0;

    /**
     * @brief Set a vector of doubles
     * @param name Parameter name
     * @param value Parameter value
     */
    virtual void setDoubleArray(const std::string& name, const std::vector<double>& value) = 0;

    /**
     * @brief Set a vector of integers
     * @param name Parameter name
     * @param value Parameter value
     */
    virtual void setIntArray(const std::string& name, const std::vector<int>& value) = 0;

    /**
     * @brief Set a vector of strings
     * @param name Parameter name
     * @param value Parameter value
     */
    virtual void setStringArray(const std::string& name, const std::vector<std::string>& value) = 0;

    /**
     * @brief Load parameters from YAML file (requires yaml-cpp)
     * @param filename Path to YAML file
     */
    virtual void loadFromYAML(const std::string& filename) = 0;
  };

  using ParameterPtr = std::shared_ptr<ParameterInterface>;

  /**
   * @brief Simple map-based parameter implementation
   * 
   * Can be used for standalone applications or testing.
   * Parameters can be loaded from YAML, JSON, or set programmatically.
   */
  class MapParameterServer : public ParameterInterface {
  public:
    MapParameterServer() = default;

    bool getBool(const std::string& name, bool default_value) override {
      auto it = bool_params_.find(name);
      return it != bool_params_.end() ? it->second : default_value;
    }

    int getInt(const std::string& name, int default_value) override {
      auto it = int_params_.find(name);
      return it != int_params_.end() ? it->second : default_value;
    }

    double getDouble(const std::string& name, double default_value) override {
      auto it = double_params_.find(name);
      return it != double_params_.end() ? it->second : default_value;
    }

    std::string getString(const std::string& name, const std::string& default_value) override {
      auto it = string_params_.find(name);
      return it != string_params_.end() ? it->second : default_value;
    }

    std::vector<double> getDoubleArray(const std::string& name, 
                                      const std::vector<double>& default_value) override {
      auto it = double_array_params_.find(name);
      return it != double_array_params_.end() ? it->second : default_value;
    }

    std::vector<int> getIntArray(const std::string& name, 
                                 const std::vector<int>& default_value) override {
      auto it = int_array_params_.find(name);
      return it != int_array_params_.end() ? it->second : default_value;
    }

    std::vector<std::string> getStringArray(const std::string& name, 
                                           const std::vector<std::string>& default_value) override {
      auto it = string_array_params_.find(name);
      return it != string_array_params_.end() ? it->second : default_value;
    }

    bool hasParameter(const std::string& name) const override {
      return bool_params_.count(name) > 0 ||
             int_params_.count(name) > 0 ||
             double_params_.count(name) > 0 ||
             string_params_.count(name) > 0 ||
             double_array_params_.count(name) > 0 ||
             int_array_params_.count(name) > 0 ||
             string_array_params_.count(name) > 0;
    }

    void setBool(const std::string& name, bool value) override { bool_params_[name] = value; }
    void setInt(const std::string& name, int value) override { int_params_[name] = value; }
    void setDouble(const std::string& name, double value) override { double_params_[name] = value; }
    void setString(const std::string& name, const std::string& value) override { string_params_[name] = value; }
    void setDoubleArray(const std::string& name, const std::vector<double>& value) override { 
      double_array_params_[name] = value; 
    }
    void setIntArray(const std::string& name, const std::vector<int>& value) override { 
      int_array_params_[name] = value; 
    }
    void setStringArray(const std::string& name, const std::vector<std::string>& value) override { 
      string_array_params_[name] = value; 
    }

    void loadFromYAML(const std::string& filename) override;

  private:
    std::map<std::string, bool> bool_params_;
    std::map<std::string, int> int_params_;
    std::map<std::string, double> double_params_;
    std::map<std::string, std::string> string_params_;
    std::map<std::string, std::vector<double>> double_array_params_;
    std::map<std::string, std::vector<int>> int_array_params_;
    std::map<std::string, std::vector<std::string>> string_array_params_;
  };

  /**
   * @brief Helper template for backwards compatibility with ROSParameter
   */
  template<typename T>
  class Parameter {
  public:
    Parameter(const std::string& name, const T& default_value, ParameterInterface& params)
      : value_(getValue(name, default_value, params)) {}

    const T& value() const { return value_; }
    operator const T&() const { return value_; }

  private:
    T value_;

    static T getValue(const std::string& name, const T& default_value, ParameterInterface& params) {
      if constexpr (std::is_same_v<T, bool>) {
        return params.getBool(name, default_value);
      } else if constexpr (std::is_same_v<T, int>) {
        return params.getInt(name, default_value);
      } else if constexpr (std::is_same_v<T, double>) {
        return params.getDouble(name, default_value);
      } else if constexpr (std::is_same_v<T, std::string>) {
        return params.getString(name, default_value);
      } else if constexpr (std::is_same_v<T, std::vector<double>>) {
        return params.getDoubleArray(name, default_value);
      } else if constexpr (std::is_same_v<T, std::vector<int>>) {
        return params.getIntArray(name, default_value);
      } else if constexpr (std::is_same_v<T, std::vector<std::string>>) {
        return params.getStringArray(name, default_value);
      } else {
        static_assert(sizeof(T) == 0, "Unsupported parameter type");
      }
    }
  };

} // namespace fgo::core

#endif // ONLINE_FGO_CORE_PARAMETER_INTERFACE_H
