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
#include "online_fgo_core/solver/FixedLagSmoother.h"
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/slam/SmartFactorParams.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
// PreintegrationType is defined in CombinedImuFactor.h, no separate header needed
#include <gtsam/navigation/CombinedImuFactor.h>
#include <mutex>
#include <map>
#include <unordered_map>

namespace fgo::graph {

/**
 * @brief IMU preintegration type (mirrors Kimera-VIO's ImuPreintegrationType)
 * 
 * This enum matches Kimera-VIO's ImuPreintegrationType exactly to ensure
 * consistent behavior between the standard VioBackend and GraphTimeCentric paths.
 */
enum class ImuPreintegrationType {
  kPreintegratedCombinedMeasurements = 0,  // Uses CombinedImuFactor (includes bias evolution)
  kPreintegratedImuMeasurements = 1        // Uses ImuFactor + separate bias BetweenFactor
};

/**
 * @brief Parameters specific to Kimera VIO integration
 * 
 * Initialization parameters match Kimera-VIO's BackendParams for consistency
 */
struct GraphTimeCentricKimeraParams {
  // State creation
  double timestampMatchTolerance = 0.001;  // seconds, for finding existing states
  bool createStatesAtIMURate = true;       // Create states at IMU measurement rate
  double imuStateFrequency = 200.0;        // Hz, if creating states at fixed rate
  
  // IMU factor configuration (mirrors Kimera-VIO ImuParams)
  // Type of IMU preintegration: 0 = CombinedImuFactor, 1 = ImuFactor
  ImuPreintegrationType imuPreintegrationType = ImuPreintegrationType::kPreintegratedCombinedMeasurements;
  // Parameters for IMU factor (from ImuParams.yaml)
  double accRandomWalk = 0.0;              // [ m / s^3 / sqrt(Hz) ] accelerometer_random_walk
  double gyroRandomWalk = 0.0;             // [ rad / s^2 / sqrt(Hz) ] gyroscope_random_walk
  double nominalSamplingTimeS = 0.005;     // 1/rate_hz from ImuParams.yaml (default 200Hz)
  
  // GP motion prior configuration  
  bool addGPMotionPriors = true;           // Add GP priors between states
  
  // Visual factor configuration
  bool enableSmartFactors = true;          // Enable smart factor management
  size_t smartFactorSlotReserve = 1000;    // Reserve slots for smart factors
  double cheiralityThreshold = 0.1;        // Minimum depth in meters
  size_t minObservations = 2;              // Minimum observations for valid landmark
  double smartFactorNoiseSigma = 1.5;      // Pixel noise sigma for smart factors
  
  // Smart factor parameters (from BackendParams)
  double rankTolerance = 1.0;              // Rank tolerance for linear triangulation
  double landmarkDistanceThreshold = 10.0; // Maximum landmark distance (m)
  double retriangulationThreshold = 0.001; // Threshold to retriangulate
  double outlierRejection = 3.0;           // Chi-squared outlier rejection threshold
  
  // Optimization
  bool optimizeOnKeyframe = true;          // Trigger optimization on keyframe arrival
  
  // Initialization parameters (matching Kimera-VIO BackendParams)
  // These control the noise models for prior factors on the first state
  // IMPORTANT: These MUST be loaded from BackendParams.yaml via initializeKimeraSupport()
  // Default values are defined in initializeKimeraSupport() to match BackendParams.yaml
  // No defaults here to ensure BackendParams.yaml is the single source of truth
  double initialPositionSigma;       // Position uncertainty (m) - loaded from BackendParams.yaml
  double initialRollPitchSigma;      // Roll/Pitch uncertainty (rad) - loaded from BackendParams.yaml
  double initialYawSigma;            // Yaw uncertainty (rad) - loaded from BackendParams.yaml
  double initialVelocitySigma;       // Velocity uncertainty (m/s) - loaded from BackendParams.yaml
  double initialAccBiasSigma;        // Accelerometer bias uncertainty (m/s²) - loaded from BackendParams.yaml
  double initialGyroBiasSigma;       // Gyroscope bias uncertainty (rad/s) - loaded from BackendParams.yaml
  
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
    using LandmarkId = int64_t;  // Match Kimera's LandmarkId type
    using FrameId = int64_t;     // Match Kimera's FrameId type
    using LandmarkIdSet = std::set<LandmarkId>;
    using SmartFactorPtr = boost::shared_ptr<gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>>;
    using SmartStereoFactorPtr = boost::shared_ptr<gtsam::SmartStereoProjectionPoseFactor>;
    using LandmarkIdSmartFactorMap = std::map<LandmarkId, SmartFactorPtr>;
    using LandmarkIdSmartStereoFactorMap = std::map<LandmarkId, SmartStereoFactorPtr>;
    
    // Slot type for factor graph slot tracking (-1 means not yet in graph)
    using Slot = int64_t;
    
    // Feature track: vector of (frame_id, stereo_measurement)
    struct StereoFeatureTrack {
        std::vector<std::pair<FrameId, gtsam::StereoPoint2>> observations;
        bool in_ba_graph = false;  // True if factor already in graph
    };
    
    // Feature track: vector of (camera_pose_key, pixel_measurement)
    struct FeatureTrack {
        std::vector<std::pair<gtsam::Key, gtsam::Point2>> observations;
        gtsam::Point3 world_position;  // Optional 3D position estimate
        bool has_3d_estimate = false;
    };
    
    // Stereo measurement type (matches Kimera)
    using StereoMeasurement = std::pair<LandmarkId, gtsam::StereoPoint2>;

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

    /**
     * @brief Convenience helper that creates (or finds) a state at timestamp and
     *        immediately sets explicit initial values.
     *
     * Mirrors Kimera-VIO's addStateValues() path to keep the integration steps
     * (state initialization vs. factor creation) decoupled.
     *
     * @param timestamp Timestamp in seconds
     * @param frame_id Kimera frame ID to use as state index (ensures key alignment with VioBackend)
     * @param pose Initial pose estimate
     * @param velocity Initial velocity estimate
     * @param bias Initial IMU bias estimate
     * @param[out] out_state_idx The created state index (valid only if function returns true)
     * @return true if successful, false otherwise
     */
    bool addKeyframeState(double timestamp,
                          size_t frame_id,
                          const gtsam::Pose3& pose,
                          const gtsam::Vector3& velocity,
                          const gtsam::imuBias::ConstantBias& bias,
                          size_t& out_state_idx);

    /**
     * @brief Initialize graph with the very first state and priors
     * @param timestamp Timestamp in seconds
     * @param frame_id Kimera frame ID to use as state index (ensures key alignment with VioBackend)
     * @param pose Initial pose estimate
     * @param velocity Initial velocity estimate
     * @param bias Initial IMU bias estimate
     * @param[out] out_state_idx The created state index (valid only if function returns true)
     * @return true if bootstrap succeeded, false otherwise
     */
    bool bootstrapInitialState(double timestamp,
                               size_t frame_id,
                               const gtsam::Pose3& pose,
                               const gtsam::Vector3& velocity,
                               const gtsam::imuBias::ConstantBias& bias,
                               size_t& out_state_idx);

    // ========================================================================
    // IMU MEASUREMENT HANDLING - Buffer and integrate IMU data
    // ========================================================================


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
     * @brief Add IMU factor between two states using preintegrated measurements (PIM)
     * 
     * This method uses a preintegrated PIM object (from Kimera frontend) directly,
     * avoiding the need to re-preintegrate raw IMU measurements.
     * 
     * @param state_i_idx Index of state i (previous keyframe)
     * @param state_j_idx Index of state j (current keyframe)
     * @param pim Preintegrated IMU measurements between states
     * @return true if factor added successfully
     */
    bool addIMUFactorFromPIM(
        size_t state_i_idx,
        size_t state_j_idx,
        const gtsam::PreintegrationType& pim);

    /**
     * @brief Add preintegrated IMU data for later factor creation
     * 
     * Stores PIM values that will be used to create IMU factors between
     * consecutive keyframe states during graph construction.
     * 
     * @param pim_data Vector of (timestamp, PIM) pairs
     *                  PIM[i] is preintegration from keyframe[i-1] to keyframe[i]
     * @return true if successfully stored
     */
    bool addPreintegratedIMUData(
        const std::vector<std::pair<double, std::shared_ptr<gtsam::PreintegrationType>>>& pim_data);

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
    /**
     * @brief Build incremental update packet without running an internal solver.
     *
     * Collects the factors, new values, and timestamps that have been added
     * since the previous optimization call. The caller is responsible for
     * feeding this packet into an external smoother (e.g. Kimera's backend).
     *
     * @param new_factors Output nonlinear factor graph containing the
     *        incremental factors.
     * @param new_values Output values containing only the newly created keys.
     * @param new_timestamps Output timestamp map for the new keys.
     * @return true if there is new information to optimize.
     */
    bool buildIncrementalUpdate(gtsam::NonlinearFactorGraph* new_factors,
                                gtsam::Values* new_values,
                                fgo::solvers::FixedLagSmoother::KeyTimestampMap* new_timestamps);

    /**
     * @brief Mark the current incremental update as consumed.
     *
     * Resets the internal tracking state so that subsequent calls to
     * buildIncrementalUpdate() only include factors/values added afterwards.
     */
    void finalizeIncrementalUpdate();

    // ========================================================================
    // RESULT RETRIEVAL - Get optimized values and covariances
    // ========================================================================

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
    // STEREO VISUAL FACTOR INTERFACE (mirrors VioBackend)
    // ========================================================================

    /**
     * @brief Set stereo camera calibration and smart factor params
     * @param stereo_cal Stereo camera calibration (K, baseline)
     * @param B_Pose_leftCam Body to left camera pose transformation
     * @param smart_noise Pre-initialized smart factor noise model (from VioBackend)
     * @param smart_params Pre-initialized smart factor params (from VioBackend)
     * 
     * Note: smart_noise and smart_params are passed from VioBackend, which initializes
     * them from BackendParams.yaml. This ensures the same parameters are used as
     * the standard VioBackend path.
     */
    void setStereoCalibration(const gtsam::Cal3_S2Stereo::shared_ptr& stereo_cal,
                              const gtsam::Pose3& B_Pose_leftCam,
                              const gtsam::SharedNoiseModel& smart_noise,
                              const gtsam::SmartProjectionParams& smart_params);

    /**
     * @brief Add stereo measurements to feature tracks and update factors
     * 
     * This is the main entry point for adding visual measurements from Kimera.
     * It mirrors VioBackend::addStereoMeasurementsToFeatureTracks + addLandmarksToGraph.
     * 
     * @param frame_id Current keyframe ID
     * @param stereo_measurements Vector of (landmark_id, stereo_point) pairs
     * @return Number of smart factors added/updated
     */
    size_t addStereoMeasurementsToGraph(
        FrameId frame_id,
        const std::vector<StereoMeasurement>& stereo_measurements);

    /**
     * @brief Add a new stereo landmark to the graph
     * @param lmk_id Landmark ID
     * @param track Feature track with stereo observations
     * @return true if successfully added
     */
    bool addStereoLandmarkToGraph(const LandmarkId& lmk_id,
                                  const StereoFeatureTrack& track);

    /**
     * @brief Update existing stereo landmark with new observation
     * @param lmk_id Landmark ID
     * @param frame_id Frame ID of new observation
     * @param measurement Stereo pixel measurement
     * @return true if successfully updated
     */
    bool updateStereoLandmarkInGraph(const LandmarkId& lmk_id,
                                     FrameId frame_id,
                                     const gtsam::StereoPoint2& measurement);

    /**
     * @brief Get all smart stereo factors for external smoother
     * @return Map of landmark IDs to smart stereo factors
     */
    const LandmarkIdSmartStereoFactorMap& getNewSmartStereoFactors() const {
        return new_smart_stereo_factors_;
    }

    /**
     * @brief Clear new smart factors after they've been consumed
     */
    void clearNewSmartStereoFactors() {
        new_smart_stereo_factors_.clear();
    }

    /**
     * @brief Delete slots for smart factors that need updating
     * @return Vector of factor slots to delete from smoother
     */
    std::vector<Slot> getSmartFactorSlotsToDelete() const;

    /**
     * @brief Update slot tracking after smoother update
     * @param lmk_ids Landmark IDs of factors added to smoother
     * @param factor_slots New slot indices assigned by smoother
     */
    void updateSmartFactorSlots(const std::vector<LandmarkId>& lmk_ids,
                                const std::vector<Slot>& factor_slots);

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

    /**
     * @brief Set initial values for an existing state from Kimera estimates
     * @param state_idx State index
     * @param pose Initial pose estimate
     * @param velocity Initial velocity estimate
     * @param bias Initial IMU bias estimate
     * @return true if values set successfully
     * 
     * This method updates the initial values for a state that was already created.
     * Used when we have better initial estimates from Kimera's backend.
     */
    bool setStateInitialValues(size_t state_idx,
                               const gtsam::Pose3& pose,
                               const gtsam::Vector3& velocity,
                               const gtsam::imuBias::ConstantBias& bias);

protected:
    // ========================================================================
    // MEMBER VARIABLES
    // ========================================================================
    
    // Kimera-specific parameters
    GraphTimeCentricKimeraParams kimeraParams_;

    // State tracking - Maps timestamp -> state index for fast lookup
    std::unordered_map<double, size_t> timestampToStateIndex_;

    // Preintegrated IMU measurements (PIM) from Kimera frontend
    // PIM[i] is preintegration from keyframe[i-1] to keyframe[i]
    // Stored as (timestamp, PIM) pairs where timestamp is the destination keyframe
    std::vector<std::pair<double, std::shared_ptr<gtsam::PreintegrationType>>> stored_pims_;

    // Track if prior factors have been added to the first state
    bool first_state_priors_added_ = false;

    // Smart factors for new landmarks (not yet added to graph)
    LandmarkIdSmartFactorMap new_smart_factors_;
    
    // Track new factors and values for incremental optimization (matching vioBackend pattern)
    // These are accumulated between optimizations and passed to solver_->update()
    gtsam::NonlinearFactorGraph new_factors_since_last_opt_;
    gtsam::Values new_values_since_last_opt_;
    fgo::solvers::FixedLagSmoother::KeyTimestampMap new_key_timestamps_since_last_opt_;
    
    // Smart factors already in the graph
    LandmarkIdSmartFactorMap old_smart_factors_;
    
    // Stereo smart factors (mirrors VioBackend structure)
    LandmarkIdSmartStereoFactorMap new_smart_stereo_factors_;
    // old_smart_stereo_factors_ stores (factor, slot) pairs where slot=-1 means not yet in graph
    std::map<LandmarkId, std::pair<SmartStereoFactorPtr, Slot>> old_smart_stereo_factors_;
    
    // Feature tracks (mirrors VioBackend::feature_tracks_)
    std::map<LandmarkId, StereoFeatureTrack> stereo_feature_tracks_;
    
    // Stereo camera calibration
    gtsam::Cal3_S2Stereo::shared_ptr stereo_cal_;
    gtsam::Pose3 B_Pose_leftCam_;  // Body to left camera transformation
    
    // Smart factor parameters (initialized from kimeraParams_)
    gtsam::SharedNoiseModel smart_noise_;
    gtsam::SmartProjectionParams smart_stereo_params_;  // SmartStereoProjectionParams is alias for SmartProjectionParams
    
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
     * @brief Helper: Add prior factors to the first state to constrain the system
     * @param state_idx State index (should be 1 for first state)
     * @param timestamp State timestamp
     * @return true if priors added successfully
     * 
     * Adds PriorFactor for pose, velocity, and bias to make the system well-constrained.
     * Uses parameters loaded from BackendParams.yaml via initializeKimeraSupport().
     * Matches Kimera-VIO's addInitialPriorFactors() approach.
     */
    bool addPriorFactorsToFirstState(size_t state_idx, double timestamp);

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
