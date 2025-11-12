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
//  Author: Kimera VIO Integration Team
//
//  GraphTimeCentricKimera: Extension of GraphTimeCentric for Kimera VIO backend integration
//  Focuses on timestamp-indexed state creation and IMU-first factor graph construction
//

#ifndef ONLINE_FGO_CORE_GRAPHTIMECENTRICKIMERA_H
#define ONLINE_FGO_CORE_GRAPHTIMECENTRICKIMERA_H

#pragma once

#include "online_fgo_core/graph/GraphTimeCentric.h"
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <mutex>
#include <map>
#include <unordered_map>
#include <optional>

namespace fgo::graph {

/**
 * @brief Parameters specific to Kimera VIO integration
 */
struct GraphTimeCentricKimeraParams {
  // State creation
  double timestampMatchTolerance = 0.001;  // seconds, for finding existing states
  bool createStatesAtIMURate = true;       // Create states at IMU measurement rate
  double imuStateFrequency = 200.0;        // Hz, if creating states at fixed rate
  
  // IMU factor configuration
  bool useCombinedIMUFactor = true;        // Use GTSAM CombinedImuFactor
  
  // GP motion prior configuration  
  bool addGPMotionPriors = true;           // Add GP priors between states
  
  // Visual factor configuration (future)
  bool enableSmartFactors = false;         // Enable smart factor management
  size_t smartFactorSlotReserve = 1000;    // Reserve slots for smart factors
  double cheiralityThreshold = 0.1;        // Minimum depth in meters
  size_t minObservations = 3;              // Minimum observations for valid landmark
  
  // Optimization
  bool optimizeOnKeyframe = true;          // Trigger optimization on keyframe arrival
  
  GraphTimeCentricKimeraParams() = default;
};

typedef std::shared_ptr<GraphTimeCentricKimeraParams> GraphTimeCentricKimeraParamsPtr;

/**
 * @brief GraphTimeCentricKimera - Extends GraphTimeCentric for Kimera VIO integration
 * 
 * Key differences from base GraphTimeCentric:
 * - States created at arbitrary timestamps (not just on IMU arrival)
 * - Timestamp-indexed state lookup for external factor insertion
 * - Interface for Kimera adapter to add measurements and factors
 * - Support for smart factor management (future phase)
 * 
 * State Creation Strategy:
 * - findOrCreateStateForTimestamp() allows state creation at any timestamp
 * - Avoids duplicate states using tolerance-based timestamp matching
 * - Uses currentPredictedBuffer_ for initial state estimates
 * 
 * IMU Factor Strategy:
 * - Reuses GraphTimeCentric's CombinedImuFactor creation
 * - Preintegrates IMU measurements between timestamp-indexed states
 * - Maintains compatibility with GraphTimeCentric optimization flow
 */
class GraphTimeCentricKimera : public GraphTimeCentric
{
public:
    typedef std::shared_ptr<GraphTimeCentricKimera> Ptr;
    
    // Type aliases for clarity
    using LandmarkId = uint64_t;
    using LandmarkIdSet = std::set<LandmarkId>;
    using SmartFactorPtr = boost::shared_ptr<gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>>;
    using LandmarkIdSmartFactorMap = std::map<LandmarkId, SmartFactorPtr>;
    
    // Feature track: vector of (camera_pose_key, pixel_measurement)
    struct FeatureTrack {
        std::vector<std::pair<gtsam::Key, gtsam::Point2>> observations;
        gtsam::Point3 world_position;  // Optional 3D position estimate
        bool has_3d_estimate = false;
    };

    /**
     * @brief Construct Kimera integration graph
     * @param app Application interface for framework-agnostic services
     */
    explicit GraphTimeCentricKimera(fgo::core::ApplicationInterface& app);

    /**
     * @brief Destructor ensuring proper cleanup
     */
    ~GraphTimeCentricKimera() override = default;

    /**
     * @brief Initialize Kimera-specific settings and parameters
     * Must be called after construction before use
     * @return true if initialization successful
     * 
     * TODO: Load Kimera-specific parameters from app interface
     * TODO: Initialize smart factor management structures
     * TODO: Configure GP prior settings
     */
    bool initializeKimeraSupport();

    // ========================================================================
    // STATE MANAGEMENT - Core functionality for timestamp-indexed states
    // ========================================================================

    /**
     * @brief Find existing state or create new state at specified timestamp
     * 
     * Algorithm:
     * 1. Search currentKeyIndexTimestampMap_ for timestamp within tolerance
     * 2. If found, return existing state index
     * 3. If not found and create_if_missing==true:
     *    a. Increment nState_
     *    b. Create keys X(nState_), V(nState_), B(nState_)
     *    c. Query predicted state from currentPredictedBuffer_ at timestamp
     *    d. Insert initial values into values_
     *    e. Update keyTimestampMap_ and currentKeyIndexTimestampMap_
     * 4. Return state index
     * 
     * @param timestamp Timestamp in seconds
     * @param create_if_missing If true, create new state if not found
     * @return State index (nState_ value), or 0 if not found and not created
     * 
     * TODO: Implement timestamp tolerance matching
     * TODO: Handle edge case when timestamp is before first state
     * TODO: Add thread safety for concurrent calls
     * TODO: Handle ValuesKeyAlreadyExists exception gracefully
     */
    size_t findOrCreateStateForTimestamp(double timestamp, bool create_if_missing = true);

    /**
     * @brief Check if a state exists at the given timestamp (within tolerance)
     * @param timestamp Timestamp in seconds
     * @param tolerance Matching tolerance in seconds (default 1ms)
     * @return true if state exists
     * 
     * TODO: Implement using currentKeyIndexTimestampMap_ search
     */
    bool hasStateAtTimestamp(double timestamp, double tolerance = 0.001) const;

    /**
     * @brief Get state index for a given timestamp
     * @param timestamp Timestamp in seconds
     * @param tolerance Matching tolerance in seconds
     * @return State index if found, 0 otherwise
     * 
     * TODO: Implement efficient lookup in currentKeyIndexTimestampMap_
     */
    size_t getStateIndexAtTimestamp(double timestamp, double tolerance = 0.001) const;

    /**
     * @brief Get all state timestamps in the current graph
     * @return Vector of timestamps (sorted)
     * 
     * TODO: Extract from keyTimestampMap_ for X keys only
     */
    std::vector<double> getAllStateTimestamps() const;

    // ========================================================================
    // IMU MEASUREMENT HANDLING - Buffer and integrate IMU data
    // ========================================================================

    /**
     * @brief Add IMU measurements to internal buffer
     * @param imu_measurements Vector of IMU measurements
     * @return true if successfully buffered
     * 
     * TODO: Buffer measurements in kimeraIMUBuffer_
     * TODO: Sort by timestamp if needed
     */
    bool addIMUMeasurements(const std::vector<fgo::data::IMUMeasurement>& imu_measurements);

    /**
     * @brief Construct factor graph from explicit list of timestamps
     * 
     * For each timestamp:
     * 1. Call findOrCreateStateForTimestamp(ts)
     * 2. Preintegrate IMU measurements between states
     * 3. Add CombinedImuFactor between consecutive states
     * 4. Optionally add GP motion prior
     * 
     * @param timestamps Vector of state timestamps in seconds
     * @return Status of graph construction
     * 
     * TODO: Implement IMU preintegration between timestamp-indexed states
     * TODO: Reuse GraphTimeCentric's imuPreIntegrationOPT_ logic
     * TODO: Handle case where no IMU data available between states
     */
    StatusGraphConstruction constructFactorGraphFromTimestamps(
        const std::vector<double>& timestamps);

    /**
     * @brief Construct IMU factors between two specific states
     * @param state_i_idx Index of state i
     * @param state_j_idx Index of state j  
     * @param imu_measurements IMU measurements to integrate
     * @return true if factor added successfully
     * 
     * TODO: Create CombinedImuFactor between the two states
     * TODO: Use preIntegratorParams_ from base class
     */
    bool addIMUFactorBetweenStates(
        size_t state_i_idx, 
        size_t state_j_idx,
        const std::vector<fgo::data::IMUMeasurement>& imu_measurements);

    // ========================================================================
    // GP MOTION PRIORS - Gaussian Process motion model
    // ========================================================================

    /**
     * @brief Add GP motion prior factors for consecutive states
     * 
     * For each consecutive pair of states in state_indices:
     * - Add GPWNOAPrior, GPWNOJPrior, or other GP prior type
     * - Use paramPtr_->gpType to determine factor type
     * - Use QcGPMotionPriorFull for noise model
     * 
     * @param state_indices Vector of state indices (must be sorted)
     * @return true if all priors added successfully
     * 
     * TODO: Implement using GraphTimeCentric::addGPMotionPrior helper
     * TODO: Extract timestamps from keyTimestampMap_ for dt calculation
     * TODO: Handle omega and acc keys if needed for WNOJ/WNOJFull
     */
    bool addGPMotionPriorsForStates(const std::vector<size_t>& state_indices);

    /**
     * @brief Add GP motion prior between two specific states
     * @param state_i_idx Index of state i
     * @param state_j_idx Index of state j
     * @return true if prior added successfully
     * 
     * TODO: Calculate dt from keyTimestampMap_
     * TODO: Call appropriate GP factor based on paramPtr_->gpType
     */
    bool addGPMotionPriorBetweenStates(size_t state_i_idx, size_t state_j_idx);

    // ========================================================================
    // EXTERNAL FACTOR INTERFACE - For Kimera adapter to add factors
    // ========================================================================

    /**
     * @brief Add an external factor (e.g., from Kimera smart factors)
     * 
     * @param factor Shared pointer to GTSAM factor
     * @param state_indices Vector of state indices this factor connects
     * @return true if factor added successfully
     * 
     * NOTE: This is for future smart factor integration
     * TODO: Validate state indices exist in graph
     * TODO: Update relatedKeys_ to prevent premature marginalization
     * TODO: Add factor to graph via push_back()
     */
    bool addExternalFactor(
        const gtsam::NonlinearFactor::shared_ptr& factor,
        const std::vector<size_t>& state_indices);

    // ========================================================================
    // OPTIMIZATION - Wrapper with external factor support
    // ========================================================================

    /**
     * @brief Optimize graph with external factors included
     * 
     * Flow:
     * 1. Ensure all external factors are added to graph
     * 2. Call GraphTimeCentric::optimize()
     * 3. Extract results and populate new_state
     * 4. Update internal state tracking
     * 
     * @param new_state Output state after optimization
     * @return Optimization time in seconds
     * 
     * TODO: Ensure external factors in relatedKeys_ are handled
     * TODO: Update Kimera-specific state tracking (landmarks, etc.)
     */
    double optimizeWithExternalFactors(fgo::data::State& new_state);

    // ========================================================================
    // RESULT RETRIEVAL - Get optimized values and covariances
    // ========================================================================

    /**
     * @brief Get optimized pose at state index
     * @param state_idx State index
     * @return Pose3 if state exists, std::nullopt otherwise
     * 
     * TODO: Query from solver_->calculateEstimate() or last result
     */
    std::optional<gtsam::Pose3> getOptimizedPose(size_t state_idx) const;

    /**
     * @brief Get optimized velocity at state index
     * @param state_idx State index
     * @return Vector3 if state exists, std::nullopt otherwise
     * 
     * TODO: Query from solver result
     */
    std::optional<gtsam::Vector3> getOptimizedVelocity(size_t state_idx) const;

    /**
     * @brief Get optimized IMU bias at state index
     * @param state_idx State index
     * @return ConstantBias if state exists, std::nullopt otherwise
     * 
     * TODO: Query from solver result
     */
    std::optional<gtsam::imuBias::ConstantBias> getOptimizedBias(size_t state_idx) const;

    /**
     * @brief Get state covariance at state index
     * @param state_idx State index
     * @return 15x15 covariance matrix (pose, vel, bias) if available
     * 
     * TODO: Use solver_->getMarginals() to extract covariance
     * TODO: Combine pose, velocity, and bias covariances
     */
    std::optional<gtsam::Matrix> getStateCovariance(size_t state_idx) const;

    // ========================================================================
    // SMART FACTOR MANAGEMENT (Existing functionality + Future Phase 3)
    // ========================================================================

    /**
     * @brief Add a new landmark to the graph as a smart factor
     * @param lm_id Unique landmark identifier from Kimera
     * @param track Feature observations across multiple frames
     * @return true if landmark was successfully added
     * 
     * TODO: Port from VioBackend::addLandmarksToGraph
     * TODO: Create SmartProjectionPoseFactor for the track
     * TODO: Add factor to graph and update tracking structures
     */
    bool addLandmarkToGraph(const LandmarkId& lm_id, const FeatureTrack& track);

    /**
     * @brief Add multiple landmarks to the graph
     * @param landmarks Set of landmark IDs to add
     * @param tracks Map of landmark IDs to feature tracks
     * @return Number of landmarks successfully added
     * 
     * TODO: Batch add landmarks with error handling
     */
    size_t addLandmarksToGraph(const LandmarkIdSet& landmarks, 
                               const std::map<LandmarkId, FeatureTrack>& tracks);

    /**
     * @brief Update existing landmark with new observations
     * @param lm_id Landmark to update
     * @param track Updated feature track
     * @return true if landmark was successfully updated
     * 
     * TODO: Update smart factor observations
     */
    bool updateLandmarkInGraph(const LandmarkId& lm_id, const FeatureTrack& track);

    /**
     * @brief Remove landmarks that fail cheirality check
     * 
     * Removes landmarks that are behind the camera or have invalid depth.
     * 
     * @param current_values Current factor graph values
     * @return Number of landmarks removed
     * 
     * TODO: Port from VioBackend::cleanCheiralityLmk
     * TODO: Check landmark depth and cheirality constraint
     * TODO: Remove invalid factors from graph
     */
    size_t cleanCheiralityLandmarks(const gtsam::Values& current_values);

    /**
     * @brief Update smart factor observation slots
     * 
     * Manages the mapping between camera pose keys and observation slots
     * in smart factors.
     * 
     * @param new_factors Smart factors to update
     * @param marginalized_keys Keys that have been marginalized
     * 
     * TODO: Port from VioBackend::updateNewSmartFactorsSlots
     * TODO: Handle slot reassignment after marginalization
     */
    void updateSmartFactorSlots(LandmarkIdSmartFactorMap& new_factors,
                                const gtsam::KeyVector& marginalized_keys);

    /**
     * @brief Get all currently tracked landmarks
     * @return Set of active landmark IDs
     */
    LandmarkIdSet getTrackedLandmarks() const;

    /**
     * @brief Remove a landmark from tracking
     * @param lm_id Landmark to remove
     * @return true if landmark was found and removed
     */
    bool removeLandmark(const LandmarkId& lm_id);

    /**
     * @brief Get statistics about landmark tracking
     * @return Map of statistic names to values
     */
    std::map<std::string, size_t> getLandmarkStatistics() const;

    // ========================================================================
    // PARAMETERS AND CONFIGURATION
    // ========================================================================

    /**
     * @brief Set Kimera-specific parameters
     * @param params Kimera parameters
     */
    void setKimeraParams(const GraphTimeCentricKimeraParams& params) {
      kimeraParams_ = params;
    }

    /**
     * @brief Get current Kimera parameters
     * @return const reference to parameters
     */
    const GraphTimeCentricKimeraParams& getKimeraParams() const {
      return kimeraParams_;
    }

protected:
    // ========================================================================
    // MEMBER VARIABLES
    // ========================================================================
    
    // Kimera-specific parameters
    GraphTimeCentricKimeraParams kimeraParams_;

    // State tracking - Maps timestamp -> state index for fast lookup
    std::unordered_map<double, size_t> timestampToStateIndex_;

    // IMU measurement buffer (separate from GraphTimeCentric's dataIMURest_)
    std::vector<fgo::data::IMUMeasurement> kimeraIMUBuffer_;

    // Smart factors for new landmarks (not yet added to graph)
    LandmarkIdSmartFactorMap new_smart_factors_;
    
    // Smart factors already in the graph
    LandmarkIdSmartFactorMap old_smart_factors_;
    
    // Landmark counter for generating internal IDs
    std::atomic<uint64_t> landmark_count_{0};
    
    // Mutex for thread-safe access to landmark data
    mutable std::mutex landmark_mutex_;
    
    // Mapping from Kimera landmark ID to internal plugin ID
    std::map<LandmarkId, uint64_t> kimera_to_plugin_id_;
    
    // ========================================================================
    // HELPER METHODS
    // ========================================================================
    
    /**
     * @brief Helper: Create initial values for a new state
     * @param state_idx State index
     * @param timestamp State timestamp
     * @return true if values created
     * 
     * TODO: Query currentPredictedBuffer_ for initial state estimate
     * TODO: Insert X, V, B keys into values_
     * TODO: Optionally insert W, A keys if GP factors enabled
     */
    bool createInitialValuesForState(size_t state_idx, double timestamp);

    /**
     * @brief Helper: Update timestamp to state index mapping
     * @param timestamp Timestamp in seconds
     * @param state_idx State index
     */
    void updateTimestampIndex(double timestamp, size_t state_idx) {
      timestampToStateIndex_[timestamp] = state_idx;
    }

    /**
     * @brief Helper: Find nearest state to timestamp
     * @param timestamp Target timestamp
     * @param tolerance Maximum time difference
     * @return State index if found within tolerance, 0 otherwise
     * 
     * TODO: Implement using currentKeyIndexTimestampMap_ iteration
     */
    size_t findNearestState(double timestamp, double tolerance) const;
    
    /**
     * @brief Check if a landmark satisfies cheirality constraint
     * @param lm_id Landmark to check
     * @param values Current graph values
     * @return true if landmark is valid (positive depth)
     * 
     * TODO: Implement depth check for landmark
     */
    bool checkLandmarkCheirality(const LandmarkId& lm_id, 
                                  const gtsam::Values& values) const;
    
    /**
     * @brief Create a smart factor from feature track
     * @param track Feature observations
     * @return Smart projection factor
     * 
     * TODO: Create SmartProjectionPoseFactor from track
     * TODO: Set noise model and calibration
     */
    SmartFactorPtr createSmartFactor(const FeatureTrack& track) const;
};

} // namespace fgo::graph

#endif //ONLINE_FGO_CORE_GRAPHTIMECENTRICKIMERA_H
