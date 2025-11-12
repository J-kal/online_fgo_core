//  Copyright 2023 Institute of Automatic Control RWTH Aachen University
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

#ifndef ONLINE_FGO_CORE_FGO_LOCALIZATION_BASE_H
#define ONLINE_FGO_CORE_FGO_LOCALIZATION_BASE_H

#pragma once

// General
#include <algorithm>
#include <map>
#include <vector>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <condition_variable>
#include <functional>
#include <GeographicLib/Geodesic.hpp>

// GTSAM
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/base/Value.h>

// Boost
#include <boost/circular_buffer.hpp>

// Online FGO Core - Interface abstraction
#include "online_fgo_core/interface/ApplicationInterface.h"
#include "online_fgo_core/interface/LoggerInterface.h"
#include "online_fgo_core/interface/ParameterInterface.h"
#include "online_fgo_core/interface/PublisherInterface.h"
#include "online_fgo_core/interface/TimeInterface.h"

// Online FGO Core - Data structures and utilities
#include "online_fgo_core/gnss_fgo_param/GNSSFGOParams.h"
#include "online_fgo_core/data/DataTypesFGO.h"
#include "online_fgo_core/data/Buffer.h"
#include "online_fgo_core/graph/GraphBase.h"
#include "online_fgo_core/graph/GraphTimeCentric.h"
#include "online_fgo_core/utils/Constants.h"
#include "online_fgo_core/utils/GNSSUtils.h"
#include "online_fgo_core/utils/NavigationTools.h"
#include "online_fgo_core/sensor/SensorCalibrationManager.h"

namespace fgo {

  // Import types from gnss_fgo namespace for convenience
  using gnss_fgo::GNSSFGOParams;
  using gnss_fgo::GNSSFGOParamsPtr;

  using gtsam::symbol_shorthand::X;  // Pose3 (R,t)
  using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
  using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
  using gtsam::symbol_shorthand::C;  // Receiver clock bias (cb,cd)
  using gtsam::symbol_shorthand::W;  // angular Velocity in body  frame
  using gtsam::symbol_shorthand::N;  // integer ambiguities
  using gtsam::symbol_shorthand::M;  // integer ddambiguities
  using gtsam::symbol_shorthand::A;  // acceleration
  using gtsam::symbol_shorthand::O;

  /**
   * @brief Framework-agnostic FGO Localization Base class
   * 
   * This class serves as the base for FGO-based localization systems.
   * It is completely ROS-agnostic and uses the ApplicationInterface
   * to interact with the underlying framework (ROS1, ROS2, or standalone).
   * 
   * Derived classes (e.g., in online_fgo_ros1) will inherit from both
   * this class and their framework-specific node class.
   */
  class FGOLocalizationBase {
  public:
    /**
     * @brief Constructor
     * @param app Application interface providing framework-specific functionality
     */
    explicit FGOLocalizationBase(fgo::core::ApplicationInterface* app);
    
    virtual ~FGOLocalizationBase();

    /**
     * @brief Get the parameter pointer
     * @return Shared pointer to GNSSFGOParams
     */
    GNSSFGOParamsPtr getParamPtr() { return paramsPtr_; }

    /**
     * @brief Get the graph pointer
     * @return Shared pointer to GraphBase
     */
    fgo::graph::GraphBase::Ptr getGraphPtr() const { return graph_; }

    /**
     * @brief Get the sensor calibration manager
     * @return Shared pointer to SensorCalibrationManager
     */
    std::shared_ptr<fgo::sensor::SensorCalibrationManager> getSensorCalibManagerPtr() { 
      return sensorCalibManager_; 
    }

    /**
     * @brief Check if state is initialized
     * @return true if initialized, false otherwise
     */
    bool isStateInitialized() const { return isStateInited_; }

  protected:
    friend class fgo::graph::GraphBase;
    friend class fgo::graph::GraphTimeCentric;

    // Application interface for framework-agnostic operations
    fgo::core::ApplicationInterface* appPtr_;

    // Parameters
    GNSSFGOParamsPtr paramsPtr_;

    // Graph and utils
    fgo::graph::GraphBase::Ptr graph_;

    // Buffer containers and utils
    fgo::core::CircularDataBuffer<fgo::data::IMUMeasurement> imuDataBuffer_;
    fgo::core::CircularDataBuffer<fgo::data::PVASolution> referenceBuffer_;
    fgo::core::CircularDataBuffer<std::array<double, 3>> initGyroBiasBuffer_;
    fgo::core::CircularDataBuffer<fgo::data::UserEstimation_T> userEstimationBuffer_;
    fgo::core::CircularDataBuffer<fgo::data::State> fgoPredStateBuffer_;
    fgo::core::CircularDataBuffer<fgo::data::State> fgoOptStateBuffer_;
    fgo::data::State lastOptimizedState_;
    fgo::data::State currentPredState_;

    // Sensor utils
    std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> currentIMUPreintegrator_;
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> preIntegratorParams_;
    std::shared_ptr<fgo::sensor::SensorCalibrationManager> sensorCalibManager_;

    // Control variables
    std::atomic_bool isDoingPropagation_ = true;
    std::atomic_bool isStateInited_{};

    std::condition_variable conDoOpt_;
    bool triggeredOpt_ = false;
    std::atomic_bool lastOptFinished_ = true;
    bool triggeredInit_ = false;
    std::atomic_bool lastInitFinished_ = true;
    fgo::core::TimeStamp lastInitTimestamp_{};
    std::condition_variable conDoInit_;

    // Threading management
    std::shared_ptr<std::thread> optThread_;
    std::shared_ptr<std::thread> initFGOThread_;
    std::mutex allBufferMutex_;
    std::mutex currentIMUPreIntMutex_;

    /**
     * @brief Initialize common variables and parameters
     * @return true if initialization successful
     */
    bool initializeCommon();

    /**
     * @brief Initialize the prior state X_0 using GNSS PVA solution
     * @return true if initialization of prior state X_0 successful
     */
    virtual bool initializeFGO();

    /**
     * @brief Callback for IMU measurements (to be implemented by derived class)
     * @param imuMeasurement IMU measurement data
     */
    virtual void onIMUMsgCb(const fgo::data::IMUMeasurement& imuMeasurement);

    /**
     * @brief Time-centric FGO loop (for continuous time-based optimization)
     * This contains the endless loop for time-centric graph construction and optimization
     */
    void timeCentricFGO();

    /**
     * @brief Time-centric FGO triggered by IMU
     * Triggers optimization when IMU is used as the timing reference
     */
    void timeCentricFGOonIMU();

    /**
     * @brief Update the graph after graph construction
     * @return runtime taken for the optimization
     */
    double optimize();

    /**
     * @brief Calculate error metrics online using interpolated GNSS PVA solution
     * @param stateIn Current FGO state
     */
    virtual void calculateErrorOnState(const fgo::data::State& stateIn);

    /**
     * @brief Trigger optimization and notify waiting threads
     */
    void notifyOptimization() {
      lastOptFinished_ = false;
      triggeredOpt_ = true;
      conDoOpt_.notify_one();
    }

    /**
     * @brief Helper functions for state publishing (framework-agnostic)
     * These can be overridden by derived classes to publish in framework-specific formats
     */
    
    /**
     * @brief Convert FGO state to a generic representation
     * Derived classes can override to publish in specific formats
     */
    virtual void publishFGOState(const fgo::data::State& state, const std::string& topic);

    /**
     * @brief Convert position to lat/lon/alt and publish
     * Derived classes can override to publish in specific formats
     */
    virtual void publishPositionAsNavFix(
      const gtsam::NavState& state,
      const fgo::core::TimeStamp& timestamp,
      const gtsam::Point3& leverArm = gtsam::Point3());

    /**
     * @brief Publish pose with covariance
     * Derived classes can override to publish in specific formats
     */
    virtual void publishPose(const fgo::data::State& state);

    /**
     * @brief Publish velocity with covariance
     * Derived classes can override to publish in specific formats
     */
    virtual void publishVelocity(const fgo::data::State& state);

    /**
     * @brief Publish timing information
     * Derived classes can override to publish in specific formats
     */
    virtual void publishTiming(double elapsedTime);

    /**
     * @brief Publish error to ground truth
     * Derived classes can override to publish in specific formats
     */
    virtual void publishError(const fgo::data::State& state);
  };

  using FGOLocalizationBasePtr = std::shared_ptr<FGOLocalizationBase>;

} // namespace fgo

#endif // ONLINE_FGO_CORE_FGO_LOCALIZATION_BASE_H
