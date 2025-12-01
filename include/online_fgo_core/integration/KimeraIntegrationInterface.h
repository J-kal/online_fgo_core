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
//  Author: Kimera VIO Integration Team
//
//  KimeraIntegrationInterface: Interface for Kimera VIO to interact with online_fgo_core
//

#ifndef ONLINE_FGO_CORE_KIMERA_INTEGRATION_INTERFACE_H
#define ONLINE_FGO_CORE_KIMERA_INTEGRATION_INTERFACE_H

#pragma once

#include <memory>
#include <vector>
#include <string>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/SmartFactorParams.h>
#include <Eigen/Dense>

#include "online_fgo_core/graph/GraphTimeCentricKimera.h"
#include "online_fgo_core/data/DataTypesFGO.h"
#include "online_fgo_core/solver/FixedLagSmoother.h"

namespace fgo::integration {

  /**
   * @brief Parameters for Kimera integration
   */
  struct KimeraIntegrationParams {
    // Graph parameters
    std::string graph_config_path;
    double smoother_lag = 5.0;  // seconds
    bool use_isam2 = true;
    
    // IMU parameters (mirrors Kimera-VIO ImuParams for consistency)
    double imu_rate = 200.0;  // Hz (rate_hz from ImuParams.yaml)
    double accel_noise_sigma = 0.01;   // accelerometer_noise_density
    double gyro_noise_sigma = 0.01;    // gyroscope_noise_density
    double accel_bias_rw_sigma = 0.0001;  // accelerometer_random_walk
    double gyro_bias_rw_sigma = 0.0001;   // gyroscope_random_walk
    Eigen::Vector3d gravity{0.0, 0.0, -9.81};
    
    // IMU preintegration type (mirrors Kimera-VIO ImuParams.yaml imu_preintegration_type)
    // 0 = kPreintegratedCombinedMeasurements (CombinedImuFactor)
    // 1 = kPreintegratedImuMeasurements (ImuFactor + bias BetweenFactor)
    int imu_preintegration_type = 0;
    
    // GP prior parameters
    bool use_gp_priors = true;
    std::string gp_type = "WNOJ";  // WNOA, WNOJ, WNOJFull, Singer, SingerFull
    
    // Optimization parameters
    bool optimize_on_keyframe = true;
    double optimization_period = 0.1;  // seconds
    
    KimeraIntegrationParams() = default;
  };

  /**
   * @brief Handle to identify a state in the graph
   * Simple wrapper around state index
   */
  struct StateHandle {
    size_t index = 0;
    double timestamp = 0.0;
    bool valid = false;
    
    StateHandle() = default;
    StateHandle(size_t idx, double ts) : index(idx), timestamp(ts), valid(true) {}
    
    explicit operator bool() const { return valid; }
  };

  /**
   * @brief Factor data for external factor insertion (future use)
   */
  struct FactorData {
    std::string factor_type;
    std::vector<StateHandle> connected_states;
    // Additional factor-specific data would go here
    
    FactorData() = default;
  };

  /**
   * @brief Interface class that Kimera adapter uses to interact with online_fgo_core
   * 
   * This interface provides a clean abstraction layer between Kimera VIO and
   * online_fgo_core's GraphTimeCentricKimera. The Kimera adapter (in Kimera-VIO package)
   * uses this interface to:
   * - Create states at arbitrary timestamps
   * - Add IMU measurements
   * - Trigger optimization
   * - Retrieve results
   * 
   * Design pattern: Facade pattern - hides complexity of GraphTimeCentricKimera
   * Thread safety: Not thread-safe, caller must synchronize if needed
   */
  class KimeraIntegrationInterface {
  public:
    struct IncrementalUpdatePacket {
      gtsam::NonlinearFactorGraph factors;
      gtsam::Values values;
      fgo::solvers::FixedLagSmoother::KeyTimestampMap key_timestamps;
    };

    /**
     * @brief Constructor
     * @param app Application interface for framework-agnostic services
     */
    explicit KimeraIntegrationInterface(fgo::core::ApplicationInterface& app);

    /**
     * @brief Destructor
     */
    virtual ~KimeraIntegrationInterface() = default;

    /**
     * @brief Initialize the integration interface with parameters
     * @param params Kimera integration parameters
     * @return true if initialization successful
     * 
     * TODO: Create GraphTimeCentricKimera instance
     * TODO: Load parameters and configure graph
     * TODO: Initialize IMU preintegration parameters
     */
    bool initialize(const KimeraIntegrationParams& params);

    /**
     * @brief Check if interface is initialized and ready
     * @return true if ready to use
     */
    bool isInitialized() const { return initialized_; }

    // ========================================================================
    // STATE MANAGEMENT
    // ========================================================================

    /**
     * @brief Create a new state at the specified timestamp
     * 
     * This will call findOrCreateStateForTimestamp on the underlying graph.
     * If a state already exists at this timestamp (within tolerance), returns
     * handle to existing state.
     * 
     * @param timestamp Timestamp in seconds
     * @return StateHandle for the created/existing state
     * 
     * TODO: Call graph_->findOrCreateStateForTimestamp()
     * TODO: Create StateHandle from result
     */
    StateHandle createStateAtTimestamp(double timestamp);

    /**
     * @brief Create a new state at the specified timestamp with initial values
     * 
     * Creates a state and sets its initial pose, velocity, and bias values.
     * This is used when we have initial estimates from Kimera's backend.
     * 
     * @param timestamp Timestamp in seconds
     * @param pose Initial pose estimate
     * @param velocity Initial velocity estimate
     * @param bias Initial IMU bias estimate
     * @return StateHandle for the created/existing state
     */
    StateHandle createStateAtTimestamp(double timestamp,
                                      const gtsam::Pose3& pose,
                                      const gtsam::Vector3& velocity,
                                      const gtsam::imuBias::ConstantBias& bias);

    /**
     * @brief Get existing state at timestamp
     * @param timestamp Timestamp in seconds
     * @return StateHandle if state exists, invalid handle otherwise
     * 
     * TODO: Call graph_->hasStateAtTimestamp() and graph_->getStateIndexAtTimestamp()
     */
    StateHandle getStateAtTimestamp(double timestamp);

    /**
     * @brief Get all state timestamps currently in the graph
     * @return Vector of timestamps in seconds
     * 
     * TODO: Call graph_->getAllStateTimestamps()
     */
    std::vector<double> getAllStateTimestamps();

    // ========================================================================
    // IMU DATA HANDLING
    // ========================================================================


    /**
     * @brief Add preintegrated IMU measurements (PIM) for keyframes (batch mode)
     * 
     * This method accepts preintegrated IMU measurements from Kimera's frontend.
     * The PIM values are stored and used to create IMU factors between consecutive
     * keyframe states during graph construction.
     * 
     * @param pim_data Vector of (timestamp, PIM) pairs
     *                  PIM[i] is preintegration from keyframe[i-1] to keyframe[i]
     * @return true if successfully added
     * 
     * @deprecated Use incremental state + factor APIs for live feeds.
     */
    bool addPreintegratedIMUData(
        const std::vector<std::pair<double, std::shared_ptr<gtsam::PreintegrationType>>>& pim_data);

    StateHandle addKeyframeState(double timestamp,
                                 size_t frame_id,
                                 const gtsam::Pose3& pose,
                                 const gtsam::Vector3& velocity,
                                 const gtsam::imuBias::ConstantBias& bias);

    StateHandle bootstrapInitialState(double timestamp,
                                      size_t frame_id,
                                      const gtsam::Pose3& pose,
                                      const gtsam::Vector3& velocity,
                                      const gtsam::imuBias::ConstantBias& bias);

    bool addImuFactorBetween(const StateHandle& previous_state,
                             const StateHandle& current_state,
                             std::shared_ptr<gtsam::PreintegrationType> pim);

    // ========================================================================
    // VISUAL FACTOR INTERFACE
    // ========================================================================

    /**
     * @brief Stereo measurement type (matches Kimera's StereoMeasurement)
     */
    using LandmarkId = int64_t;
    using FrameId = int64_t;
    using StereoMeasurement = std::pair<LandmarkId, gtsam::StereoPoint2>;

    /**
     * @brief Set stereo camera calibration and smart factor params for visual factors
     * @param stereo_cal Stereo camera calibration (K, baseline)
     * @param B_Pose_leftCam Body to left camera pose transformation
     * @param smart_noise Pre-initialized smart factor noise model (from VioBackend)
     * @param smart_params Pre-initialized smart factor params (from VioBackend)
     */
    void setStereoCalibration(const gtsam::Cal3_S2Stereo::shared_ptr& stereo_cal,
                              const gtsam::Pose3& B_Pose_leftCam,
                              const gtsam::SharedNoiseModel& smart_noise,
                              const gtsam::SmartProjectionParams& smart_params);

    /**
     * @brief Add stereo visual measurements for a keyframe
     * 
     * Processes stereo measurements and adds/updates SmartStereoFactors
     * in the underlying graph.
     * 
     * @param frame_id Current keyframe ID
     * @param stereo_measurements Vector of (landmark_id, stereo_point) pairs
     * @return Number of landmarks added/updated
     */
    size_t addStereoMeasurements(FrameId frame_id,
                                 const std::vector<StereoMeasurement>& stereo_measurements);

    // ========================================================================
    // FACTOR INSERTION (Future use for visual factors)
    // ========================================================================

    /**
     * @brief Add an external factor to the graph
     * @param factor_data Data describing the factor
     * @return true if successfully added
     * 
     * NOTE: This is a placeholder for future smart factor integration
     * TODO: Implement when adding visual factors
     */
    bool addFactor(const FactorData& factor_data);

    // ========================================================================
    // OPTIMIZATION
    // ========================================================================

    bool buildIncrementalUpdate(IncrementalUpdatePacket* packet);
    void markIncrementalUpdateConsumed();

    // ========================================================================
    // RESULT RETRIEVAL
    // ========================================================================

    // ========================================================================
    // CONFIGURATION
    // ========================================================================

    /**
     * @brief Update integration parameters at runtime
     * @param params New parameters
     * 
     * TODO: Update graph_->setKimeraParams() with relevant params
     */
    void updateParameters(const KimeraIntegrationParams& params);

    /**
     * @brief Get current parameters
     * @return Current integration parameters
     */
    const KimeraIntegrationParams& getParameters() const { return params_; }

    /**
     * @brief Get pointer to underlying graph (for advanced use)
     * @return Pointer to GraphTimeCentricKimera
     * 
     * NOTE: Use with caution, prefer using interface methods
     */
    std::shared_ptr<fgo::graph::GraphTimeCentricKimera> getGraph() { return graph_; }

  protected:
    // Application interface
    fgo::core::ApplicationInterface* app_;
    
    // Underlying graph
    std::shared_ptr<fgo::graph::GraphTimeCentricKimera> graph_;
    
    // Parameters
    KimeraIntegrationParams params_;
    
    // Initialization flag
    bool initialized_ = false;

    // Buffered state timestamps (for batched optimization)
    std::vector<double> bufferedStateTimestamps_;
    
    // Last processed keyframe handle (for convenience helpers)
    StateHandle last_keyframe_handle_;
    
  };

} // namespace fgo::integration

#endif // ONLINE_FGO_CORE_KIMERA_INTEGRATION_INTERFACE_H
