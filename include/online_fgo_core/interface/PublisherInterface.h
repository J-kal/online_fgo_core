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

#ifndef ONLINE_FGO_CORE_PUBLISHER_INTERFACE_H
#define ONLINE_FGO_CORE_PUBLISHER_INTERFACE_H

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "TimeInterface.h"

namespace fgo::core {

  /**
   * @brief Abstract interface for publishing messages
   * 
   * This templated interface allows the core FGO library to publish
   * data without depending on ROS publishers.
   * 
   * @tparam MsgType The message type to publish
   */
  template<typename MsgType>
  class PublisherInterface {
  public:
    virtual ~PublisherInterface() = default;

    /**
     * @brief Publish a message
     * @param msg The message to publish
     */
    virtual void publish(const MsgType& msg) = 0;

    /**
     * @brief Get the number of subscribers
     * @return Number of active subscribers
     */
    virtual size_t getNumSubscribers() const { return 0; }
  };

  template<typename MsgType>
  using PublisherPtr = std::shared_ptr<PublisherInterface<MsgType>>;

  /**
   * @brief Core message types (ROS-agnostic)
   */

  struct SensorProcessingReport {
    TimeStamp timestamp;
    std::string sensor_name;
    int num_factors_added;
    int num_measurements_processed;
    double processing_time_ms;
    bool optimization_triggered;
    std::string status_message;
  };

  struct FactorResidual {
    uint64_t factor_id;
    std::string factor_type;
    double residual;
    int dimension;
    std::vector<uint64_t> connected_keys;
  };

  struct FactorResidualsMsg {
    TimeStamp timestamp;
    uint64_t state_index;
    std::vector<FactorResidual> residuals;
    double total_cost;
    int num_factors;
  };

  struct FGOState {
    TimeStamp timestamp;
    uint64_t state_index;
    
    // Position (ECEF or local frame)
    double position_x;
    double position_y;
    double position_z;
    std::vector<double> position_covariance; // 3x3 flattened
    
    // Orientation (quaternion)
    double orientation_w;
    double orientation_x;
    double orientation_y;
    double orientation_z;
    std::vector<double> orientation_covariance; // 3x3 flattened
    
    // Velocity
    double velocity_x;
    double velocity_y;
    double velocity_z;
    std::vector<double> velocity_covariance; // 3x3 flattened
    
    // IMU bias
    double acc_bias_x;
    double acc_bias_y;
    double acc_bias_z;
    double gyro_bias_x;
    double gyro_bias_y;
    double gyro_bias_z;
    std::vector<double> bias_covariance; // 6x6 flattened
    
    // Clock bias (for GNSS)
    double clock_bias;
    double clock_drift;
    std::vector<double> clock_covariance; // 2x2 flattened
    
    // State type
    std::string state_type; // "predicted", "optimized", "extrapolated"
  };

  struct PoseWithCovarianceMsg {
    TimeStamp timestamp;
    
    // Position
    double position_x;
    double position_y;
    double position_z;
    
    // Orientation (quaternion)
    double orientation_w;
    double orientation_x;
    double orientation_y;
    double orientation_z;
    
    // Covariance (6x6 flattened: xyz rpy)
    std::vector<double> covariance;
  };

  struct TwistWithCovarianceMsg {
    TimeStamp timestamp;
    
    // Linear velocity
    double linear_x;
    double linear_y;
    double linear_z;
    
    // Angular velocity
    double angular_x;
    double angular_y;
    double angular_z;
    
    // Covariance (6x6 flattened)
    std::vector<double> covariance;
  };

  /**
   * @brief Null publisher that does nothing
   * 
   * Used when publishing is disabled or not needed
   */
  template<typename MsgType>
  class NullPublisher : public PublisherInterface<MsgType> {
  public:
    void publish(const MsgType&) override {}
    size_t getNumSubscribers() const override { return 0; }
  };

} // namespace fgo::core

#endif // ONLINE_FGO_CORE_PUBLISHER_INTERFACE_H
