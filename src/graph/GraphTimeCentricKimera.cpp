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
//  GraphTimeCentricKimera implementation
//

#include "online_fgo_core/graph/GraphTimeCentricKimera.h"
#include "online_fgo_core/graph/GraphUtils.h"
#include <algorithm>
#include <cmath>

namespace fgo::graph {

GraphTimeCentricKimera::GraphTimeCentricKimera(fgo::core::ApplicationInterface& app)
    : GraphTimeCentric(app) {
  appPtr_->getLogger().info("GraphTimeCentricKimera: initializing...");
  
  // Initialize Kimera-specific parameters with defaults
  kimeraParams_ = GraphTimeCentricKimeraParams();
  
  appPtr_->getLogger().info("GraphTimeCentricKimera: initialized!");
}

bool GraphTimeCentricKimera::initializeKimeraSupport() {
  appPtr_->getLogger().info("GraphTimeCentricKimera: initializing Kimera support...");
  std::cout << "[online_fgo_core] GraphTimeCentricKimera: INITIALIZING KIMERA SUPPORT" << std::endl;
  
  try {
    // Load Kimera-specific parameters from application interface
    auto& params = appPtr_->getParameters();
    
    // State creation parameters
    kimeraParams_.timestampMatchTolerance = params.getDouble("kimera.timestamp_match_tolerance", 0.001);
    kimeraParams_.createStatesAtIMURate = params.getBool("kimera.create_states_at_imu_rate", true);
    kimeraParams_.imuStateFrequency = params.getDouble("kimera.imu_state_frequency", 200.0);
    
    // IMU factor configuration
    kimeraParams_.useCombinedIMUFactor = params.getBool("kimera.use_combined_imu_factor", true);
    
    // GP prior configuration
    kimeraParams_.addGPMotionPriors = params.getBool("kimera.add_gp_motion_priors", true);
    
    // Smart factor configuration (future)
    kimeraParams_.enableSmartFactors = params.getBool("kimera.enable_smart_factors", false);
    kimeraParams_.cheiralityThreshold = params.getDouble("kimera.cheirality_threshold", 0.1);
    kimeraParams_.minObservations = params.getInt("kimera.min_observations", 2);
    
    appPtr_->getLogger().info("GraphTimeCentricKimera: Kimera support initialized successfully");
    appPtr_->getLogger().info("  - Timestamp tolerance: " + std::to_string(kimeraParams_.timestampMatchTolerance) + " s");
    appPtr_->getLogger().info("  - Use combined IMU factor: " + std::string(kimeraParams_.useCombinedIMUFactor ? "true" : "false"));
    appPtr_->getLogger().info("  - Add GP motion priors: " + std::string(kimeraParams_.addGPMotionPriors ? "true" : "false"));
    
    return true;
    
  } catch (const std::exception& e) {
    appPtr_->getLogger().error("GraphTimeCentricKimera: Failed to initialize Kimera support: " + std::string(e.what()));
    return false;
  }
}

// ========================================================================
// STATE MANAGEMENT
// ========================================================================

size_t GraphTimeCentricKimera::findOrCreateStateForTimestamp(double timestamp, bool create_if_missing) {
  std::cout << "[online_fgo_core] GraphTimeCentricKimera: FIND OR CREATE STATE FOR TIMESTAMP " << timestamp << std::endl;
  // 1. Search for existing state within tolerance
  size_t existing_state = findNearestState(timestamp, kimeraParams_.timestampMatchTolerance);
  
  if (existing_state != 0) {
    appPtr_->getLogger().debug("GraphTimeCentricKimera: Found existing state " + 
                               std::to_string(existing_state) + " at timestamp " + 
                               std::to_string(timestamp));
    return existing_state;
  }
  
  // 2. If not found and create_if_missing is false, return 0
  if (!create_if_missing) {
    appPtr_->getLogger().warn("GraphTimeCentricKimera: No state found at timestamp " + 
                              std::to_string(timestamp) + " and create_if_missing=false");
    return 0;
  }
  
  // 3. Create new state
  nState_++;
  
  appPtr_->getLogger().info("GraphTimeCentricKimera: Creating new state " + 
                            std::to_string(nState_) + " at timestamp " + 
                            std::to_string(timestamp));
  
  // 4. Create initial values for the new state
  if (!createInitialValuesForState(nState_, timestamp)) {
    appPtr_->getLogger().error("GraphTimeCentricKimera: Failed to create initial values for state " + 
                               std::to_string(nState_));
    nState_--;  // Rollback
    return 0;
  }
  
  // 5. Update timestamp mapping
  updateTimestampIndex(timestamp, nState_);
  
  return nState_;
}

bool GraphTimeCentricKimera::hasStateAtTimestamp(double timestamp, double tolerance) const {
  size_t state_idx = findNearestState(timestamp, tolerance);
  return (state_idx != 0);
}

size_t GraphTimeCentricKimera::getStateIndexAtTimestamp(double timestamp, double tolerance) const {
  return findNearestState(timestamp, tolerance);
}

std::vector<double> GraphTimeCentricKimera::getAllStateTimestamps() const {
  std::vector<double> timestamps;
  
  // Extract timestamps from keyTimestampMap_ for X keys only
  for (const auto& [key, ts] : keyTimestampMap_) {
    // Check if this is a pose key (X symbol)
    if (gtsam::Symbol(key).chr() == 'x') {
      timestamps.push_back(ts);
    }
  }
  
  // Sort timestamps
  std::sort(timestamps.begin(), timestamps.end());
  
  return timestamps;
}

// ========================================================================
// IMU MEASUREMENT HANDLING
// ========================================================================

bool GraphTimeCentricKimera::addIMUMeasurements(
    const std::vector<fgo::data::IMUMeasurement>& imu_measurements) {
  
  if (imu_measurements.empty()) {
    appPtr_->getLogger().warn("GraphTimeCentricKimera: Attempted to add empty IMU measurements");
    return false;
  }
  
  std::cout << "[online_fgo_core] GraphTimeCentricKimera: ADDING " << imu_measurements.size() << " IMU MEASUREMENTS" << std::endl;
  
  appPtr_->getLogger().debug("GraphTimeCentricKimera: Adding " + 
                             std::to_string(imu_measurements.size()) + " IMU measurements");
  
  // Add to Kimera IMU buffer
  kimeraIMUBuffer_.insert(kimeraIMUBuffer_.end(), 
                          imu_measurements.begin(), 
                          imu_measurements.end());
  
  // Sort by timestamp to ensure chronological order
  std::sort(kimeraIMUBuffer_.begin(), kimeraIMUBuffer_.end(),
            [](const fgo::data::IMUMeasurement& a, const fgo::data::IMUMeasurement& b) {
              return a.timestamp.seconds() < b.timestamp.seconds();
            });
  
  return true;
}

StatusGraphConstruction GraphTimeCentricKimera::constructFactorGraphFromTimestamps(
    const std::vector<double>& timestamps) {
  
  if (timestamps.empty()) {
    appPtr_->getLogger().error("GraphTimeCentricKimera: No timestamps provided for graph construction");
    return StatusGraphConstruction::FAILED;
  }
  
  std::cout << "[online_fgo_core] GraphTimeCentricKimera: CONSTRUCT FACTOR GRAPH FROM " << timestamps.size() << " TIMESTAMPS" << std::endl;
  
  appPtr_->getLogger().info("GraphTimeCentricKimera: Constructing factor graph from " + 
                            std::to_string(timestamps.size()) + " timestamps");
  
  // Ensure we have IMU data
  if (kimeraIMUBuffer_.empty()) {
    appPtr_->getLogger().warn("GraphTimeCentricKimera: No IMU data available for preintegration");
  }
  
  std::vector<size_t> created_states;
  
  // Create states at each timestamp
  for (const auto& ts : timestamps) {
    size_t state_idx = findOrCreateStateForTimestamp(ts, true);
    
    if (state_idx == 0) {
      appPtr_->getLogger().error("GraphTimeCentricKimera: Failed to create state at timestamp " + 
                                 std::to_string(ts));
      return StatusGraphConstruction::FAILED;
    }
    
    created_states.push_back(state_idx);
  }
  
  // Create IMU factors between consecutive states
  for (size_t i = 1; i < created_states.size(); ++i) {
    size_t state_i = created_states[i-1];
    size_t state_j = created_states[i];
    
    double ts_i = keyTimestampMap_[X(state_i)];
    double ts_j = keyTimestampMap_[X(state_j)];
    
    // Extract IMU measurements between these timestamps
    std::vector<fgo::data::IMUMeasurement> imu_between;
    for (const auto& imu : kimeraIMUBuffer_) {
      double imu_ts = imu.timestamp.seconds();
      if (imu_ts >= ts_i && imu_ts < ts_j) {
        imu_between.push_back(imu);
      }
    }
    
    if (imu_between.empty()) {
      appPtr_->getLogger().warn("GraphTimeCentricKimera: No IMU data between states " + 
                                std::to_string(state_i) + " and " + std::to_string(state_j));
      continue;
    }
    
    // Add IMU factor
    if (!addIMUFactorBetweenStates(state_i, state_j, imu_between)) {
      appPtr_->getLogger().error("GraphTimeCentricKimera: Failed to add IMU factor between states " + 
                                 std::to_string(state_i) + " and " + std::to_string(state_j));
      return StatusGraphConstruction::FAILED;
    }
  }
  
  // Add GP motion priors if enabled
  if (kimeraParams_.addGPMotionPriors) {
    if (!addGPMotionPriorsForStates(created_states)) {
      appPtr_->getLogger().error("GraphTimeCentricKimera: Failed to add GP motion priors");
      return StatusGraphConstruction::FAILED;
    }
  }
  
  appPtr_->getLogger().info("GraphTimeCentricKimera: Factor graph construction successful");
  return StatusGraphConstruction::SUCCESSFUL;
}

bool GraphTimeCentricKimera::addIMUFactorBetweenStates(
    size_t state_i_idx, 
    size_t state_j_idx,
    const std::vector<fgo::data::IMUMeasurement>& imu_measurements) {
  
  if (imu_measurements.empty()) {
    appPtr_->getLogger().warn("GraphTimeCentricKimera: No IMU measurements to integrate");
    return false;
  }
  
  try {
    // Get keys for states
    gtsam::Key pose_i = X(state_i_idx);
    gtsam::Key vel_i = V(state_i_idx);
    gtsam::Key bias_i = B(state_i_idx);
    gtsam::Key pose_j = X(state_j_idx);
    gtsam::Key vel_j = V(state_j_idx);
    gtsam::Key bias_j = B(state_j_idx);
    
    // Get current bias estimate from values
    gtsam::imuBias::ConstantBias current_bias;
    if (values_.exists(bias_i)) {
      current_bias = values_.at<gtsam::imuBias::ConstantBias>(bias_i);
    } else {
      // Use zero bias if not available
      current_bias = gtsam::imuBias::ConstantBias();
      appPtr_->getLogger().warn("GraphTimeCentricKimera: No bias available for state " + 
                                std::to_string(state_i_idx) + ", using zero bias");
    }
    
    // Create preintegrator with current bias
    std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> pim = 
        std::make_shared<gtsam::PreintegratedCombinedMeasurements>(preIntegratorParams_, current_bias);
    
    // Integrate IMU measurements
    for (const auto& imu : imu_measurements) {
      pim->integrateMeasurement(imu.accLin, imu.gyro, imu.dt);
    }
    
    // Create combined IMU factor
    boost::shared_ptr<gtsam::CombinedImuFactor> imu_factor =
        boost::make_shared<gtsam::CombinedImuFactor>(pose_i, vel_i,
                                                     pose_j, vel_j,
                                                     bias_i, bias_j,
                                                     *pim);
    
    // Add factor to graph
    this->push_back(imu_factor);
    
    appPtr_->getLogger().debug("GraphTimeCentricKimera: Added IMU factor between states " + 
                               std::to_string(state_i_idx) + " and " + 
                               std::to_string(state_j_idx) + " with " + 
                               std::to_string(imu_measurements.size()) + " measurements");
    
    return true;
    
  } catch (const std::exception& e) {
    appPtr_->getLogger().error("GraphTimeCentricKimera: Exception while adding IMU factor: " + 
                               std::string(e.what()));
    return false;
  }
}

// ========================================================================
// GP MOTION PRIORS
// ========================================================================

bool GraphTimeCentricKimera::addGPMotionPriorsForStates(const std::vector<size_t>& state_indices) {
  if (state_indices.size() < 2) {
    appPtr_->getLogger().warn("GraphTimeCentricKimera: Need at least 2 states for GP priors");
    return false;
  }
  
  appPtr_->getLogger().info("GraphTimeCentricKimera: Adding GP motion priors for " + 
                            std::to_string(state_indices.size() - 1) + " state pairs");
  
  size_t priors_added = 0;
  
  for (size_t i = 1; i < state_indices.size(); ++i) {
    if (addGPMotionPriorBetweenStates(state_indices[i-1], state_indices[i])) {
      priors_added++;
    }
  }
  
  appPtr_->getLogger().info("GraphTimeCentricKimera: Successfully added " + 
                            std::to_string(priors_added) + " GP motion priors");
  
  return (priors_added > 0);
}

bool GraphTimeCentricKimera::addGPMotionPriorBetweenStates(size_t state_i_idx, size_t state_j_idx) {
  try {
    // Get keys
    gtsam::Key pose_i = X(state_i_idx);
    gtsam::Key vel_i = V(state_i_idx);
    gtsam::Key omega_i = W(state_i_idx);
    gtsam::Key acc_i = A(state_i_idx);
    gtsam::Key pose_j = X(state_j_idx);
    gtsam::Key vel_j = V(state_j_idx);
    gtsam::Key omega_j = W(state_j_idx);
    gtsam::Key acc_j = A(state_j_idx);
    
    // Calculate dt
    double ts_i = keyTimestampMap_[pose_i];
    double ts_j = keyTimestampMap_[pose_j];
    double dt = ts_j - ts_i;
    
    if (dt <= 0.0) {
      appPtr_->getLogger().error("GraphTimeCentricKimera: Invalid dt " + std::to_string(dt) + 
                                 " between states " + std::to_string(state_i_idx) + 
                                 " and " + std::to_string(state_j_idx));
      return false;
    }
    
    // Get acceleration values if needed for WNOJ/WNOJFull
    gtsam::Vector6 acc_i_val = gtsam::Vector6::Zero();
    gtsam::Vector6 acc_j_val = gtsam::Vector6::Zero();
    
    if (graphBaseParamPtr_->gpType == fgo::data::GPModelType::WNOJ || 
        graphBaseParamPtr_->gpType == fgo::data::GPModelType::WNOJFull ||
        graphBaseParamPtr_->gpType == fgo::data::GPModelType::Singer ||
        graphBaseParamPtr_->gpType == fgo::data::GPModelType::SingerFull) {
      
      // Try to get from accBuffer or use zero
      if (accBuffer_.size() > state_i_idx) {
        acc_i_val = accBuffer_.get_buffer_from_id(state_i_idx);
      }
      
      if (accBuffer_.size() > state_j_idx) {
        acc_j_val = accBuffer_.get_buffer_from_id(state_j_idx);
      }
    }
    
    // Call base class helper to add GP prior
    gtsam::Matrix6 ad = gtsam::Matrix6::Identity();
    
    this->addGPMotionPrior(pose_i, vel_i, omega_i, acc_i,
                          pose_j, vel_j, omega_j, acc_j,
                          dt, acc_i_val, acc_j_val, ad);
    
    appPtr_->getLogger().debug("GraphTimeCentricKimera: Added GP motion prior between states " + 
                               std::to_string(state_i_idx) + " and " + 
                               std::to_string(state_j_idx) + " (dt=" + std::to_string(dt) + ")");
    
    return true;
    
  } catch (const std::exception& e) {
    appPtr_->getLogger().error("GraphTimeCentricKimera: Exception while adding GP prior: " + 
                               std::string(e.what()));
    return false;
  }
}

// ========================================================================
// EXTERNAL FACTOR INTERFACE
// ========================================================================

bool GraphTimeCentricKimera::addExternalFactor(
    const gtsam::NonlinearFactor::shared_ptr& factor,
    const std::vector<size_t>& state_indices) {
  
  if (!factor) {
    appPtr_->getLogger().error("GraphTimeCentricKimera: Null factor provided");
    return false;
  }
  
  // Validate that all state indices exist
  for (size_t idx : state_indices) {
    if (!values_.exists(X(idx))) {
      appPtr_->getLogger().error("GraphTimeCentricKimera: State " + std::to_string(idx) + 
                                 " does not exist in values");
      return false;
    }
  }
  
  // Add factor to graph
  this->push_back(factor);
  
  // Update relatedKeys_ to prevent premature marginalization
  for (size_t idx : state_indices) {
    relatedKeys_.push_back(X(idx));
    relatedKeys_.push_back(V(idx));
    relatedKeys_.push_back(B(idx));
  }
  
  appPtr_->getLogger().debug("GraphTimeCentricKimera: Added external factor connecting " + 
                             std::to_string(state_indices.size()) + " states");
  
  return true;
}

// ========================================================================
// OPTIMIZATION
// ========================================================================

double GraphTimeCentricKimera::optimizeWithExternalFactors(fgo::data::State& new_state) {
  appPtr_->getLogger().info("GraphTimeCentricKimera: Starting optimization with external factors...");
  std::cout << "[online_fgo_core] GraphTimeCentricKimera: OPTIMIZE WITH EXTERNAL FACTORS" << std::endl;
  
  // Call base class optimize method
  double opt_time = this->optimize(new_state);
  
  // Clear related keys after optimization
  relatedKeys_.clear();
  
  appPtr_->getLogger().info("GraphTimeCentricKimera: Optimization completed in " + 
                            std::to_string(opt_time) + " seconds");
  
  return opt_time;
}

// ========================================================================
// RESULT RETRIEVAL
// ========================================================================

std::optional<gtsam::Pose3> GraphTimeCentricKimera::getOptimizedPose(size_t state_idx) const {
  try {
    gtsam::Values result = solver_->calculateEstimate();
    gtsam::Key pose_key = X(state_idx);
    
    if (result.exists(pose_key)) {
      return result.at<gtsam::Pose3>(pose_key);
    }
    
    appPtr_->getLogger().warn("GraphTimeCentricKimera: Pose not found for state " + 
                              std::to_string(state_idx));
    return std::nullopt;
    
  } catch (const std::exception& e) {
    appPtr_->getLogger().error("GraphTimeCentricKimera: Exception getting optimized pose: " + 
                               std::string(e.what()));
    return std::nullopt;
  }
}

std::optional<gtsam::Vector3> GraphTimeCentricKimera::getOptimizedVelocity(size_t state_idx) const {
  try {
    gtsam::Values result = solver_->calculateEstimate();
    gtsam::Key vel_key = V(state_idx);
    
    if (result.exists(vel_key)) {
      return result.at<gtsam::Vector3>(vel_key);
    }
    
    appPtr_->getLogger().warn("GraphTimeCentricKimera: Velocity not found for state " + 
                              std::to_string(state_idx));
    return std::nullopt;
    
  } catch (const std::exception& e) {
    appPtr_->getLogger().error("GraphTimeCentricKimera: Exception getting optimized velocity: " + 
                               std::string(e.what()));
    return std::nullopt;
  }
}

std::optional<gtsam::imuBias::ConstantBias> GraphTimeCentricKimera::getOptimizedBias(size_t state_idx) const {
  try {
    gtsam::Values result = solver_->calculateEstimate();
    gtsam::Key bias_key = B(state_idx);
    
    if (result.exists(bias_key)) {
      return result.at<gtsam::imuBias::ConstantBias>(bias_key);
    }
    
    appPtr_->getLogger().warn("GraphTimeCentricKimera: Bias not found for state " + 
                              std::to_string(state_idx));
    return std::nullopt;
    
  } catch (const std::exception& e) {
    appPtr_->getLogger().error("GraphTimeCentricKimera: Exception getting optimized bias: " + 
                               std::string(e.what()));
    return std::nullopt;
  }
}

std::optional<gtsam::Matrix> GraphTimeCentricKimera::getStateCovariance(size_t state_idx) const {
  try {
    gtsam::Values result = solver_->calculateEstimate();
    gtsam::Marginals marginals(*this, result);
    
    gtsam::Key pose_key = X(state_idx);
    gtsam::Key vel_key = V(state_idx);
    gtsam::Key bias_key = B(state_idx);
    
    if (!result.exists(pose_key) || !result.exists(vel_key) || !result.exists(bias_key)) {
      appPtr_->getLogger().warn("GraphTimeCentricKimera: Not all keys exist for state " + 
                                std::to_string(state_idx));
      return std::nullopt;
    }
    
    // Get individual covariances
    gtsam::Matrix pose_cov = marginals.marginalCovariance(pose_key);
    gtsam::Matrix vel_cov = marginals.marginalCovariance(vel_key);
    gtsam::Matrix bias_cov = marginals.marginalCovariance(bias_key);
    
    // Combine into 15x15 matrix (6+3+6)
    gtsam::Matrix combined_cov = gtsam::Matrix::Zero(15, 15);
    combined_cov.block<6, 6>(0, 0) = pose_cov;
    combined_cov.block<3, 3>(6, 6) = vel_cov;
    combined_cov.block<6, 6>(9, 9) = bias_cov;
    
    return combined_cov;
    
  } catch (const std::exception& e) {
    appPtr_->getLogger().error("GraphTimeCentricKimera: Exception getting state covariance: " + 
                               std::string(e.what()));
    return std::nullopt;
  }
}

// ========================================================================
// SMART FACTOR MANAGEMENT (Stubs for future implementation)
// ========================================================================

bool GraphTimeCentricKimera::addLandmarkToGraph(const LandmarkId& lm_id, const FeatureTrack& track) {
  // TODO: Implement in Phase 3
  appPtr_->getLogger().warn("GraphTimeCentricKimera: addLandmarkToGraph not yet implemented");
  return false;
}

size_t GraphTimeCentricKimera::addLandmarksToGraph(
    const LandmarkIdSet& landmarks, 
    const std::map<LandmarkId, FeatureTrack>& tracks) {
  // TODO: Implement in Phase 3
  appPtr_->getLogger().warn("GraphTimeCentricKimera: addLandmarksToGraph not yet implemented");
  return 0;
}

bool GraphTimeCentricKimera::updateLandmarkInGraph(const LandmarkId& lm_id, const FeatureTrack& track) {
  // TODO: Implement in Phase 3
  appPtr_->getLogger().warn("GraphTimeCentricKimera: updateLandmarkInGraph not yet implemented");
  return false;
}

size_t GraphTimeCentricKimera::cleanCheiralityLandmarks(const gtsam::Values& current_values) {
  // TODO: Implement in Phase 3
  appPtr_->getLogger().warn("GraphTimeCentricKimera: cleanCheiralityLandmarks not yet implemented");
  return 0;
}

void GraphTimeCentricKimera::updateSmartFactorSlots(
    LandmarkIdSmartFactorMap& new_factors,
    const gtsam::KeyVector& marginalized_keys) {
  // TODO: Implement in Phase 3
  appPtr_->getLogger().warn("GraphTimeCentricKimera: updateSmartFactorSlots not yet implemented");
}

GraphTimeCentricKimera::LandmarkIdSet GraphTimeCentricKimera::getTrackedLandmarks() const {
  // TODO: Implement in Phase 3
  return LandmarkIdSet();
}

bool GraphTimeCentricKimera::removeLandmark(const LandmarkId& lm_id) {
  // TODO: Implement in Phase 3
  appPtr_->getLogger().warn("GraphTimeCentricKimera: removeLandmark not yet implemented");
  return false;
}

std::map<std::string, size_t> GraphTimeCentricKimera::getLandmarkStatistics() const {
  // TODO: Implement in Phase 3
  return std::map<std::string, size_t>();
}

// ========================================================================
// HELPER METHODS
// ========================================================================

bool GraphTimeCentricKimera::createInitialValuesForState(size_t state_idx, double timestamp) {
  try {
    // Query predicted state from currentPredictedBuffer_
    auto predicted_state = graph::queryCurrentPredictedState(
        currentPredictedBuffer_.get_all_time_buffer_pair(), timestamp);
    
    // Create keys
    gtsam::Key pose_key = X(state_idx);
    gtsam::Key vel_key = V(state_idx);
    gtsam::Key bias_key = B(state_idx);
    
    // Insert initial values
    values_.insert(pose_key, predicted_state.state.pose());
    values_.insert(vel_key, predicted_state.state.velocity());
    values_.insert(bias_key, predicted_state.imuBias);
    
    // Update keyTimestampMap
    keyTimestampMap_[pose_key] = timestamp;
    keyTimestampMap_[vel_key] = timestamp;
    keyTimestampMap_[bias_key] = timestamp;
    
    // Update currentKeyIndexTimestampMap_
    currentKeyIndexTimestampMap_.insert(std::make_pair(state_idx, timestamp));
    
    // If GP factors enabled, also insert omega and acc keys
    if (graphBaseParamPtr_->addGPPriorFactor || graphBaseParamPtr_->addGPInterpolatedFactor) {
      gtsam::Key omega_key = W(state_idx);
      values_.insert(omega_key, predicted_state.omega);
      keyTimestampMap_[omega_key] = timestamp;
    }
    
    if (graphBaseParamPtr_->gpType == fgo::data::GPModelType::WNOJFull || 
        graphBaseParamPtr_->gpType == fgo::data::GPModelType::SingerFull ||
        graphBaseParamPtr_->addConstantAccelerationFactor) {
      gtsam::Key acc_key = A(state_idx);
      
      // Get acceleration from buffer if available, otherwise use zero
      gtsam::Vector6 acc = gtsam::Vector6::Zero();
      if (state_idx > 0 && accBuffer_.size() > (state_idx - 1)) {
        acc = accBuffer_.get_buffer_from_id(state_idx - 1);
      }
      
      values_.insert(acc_key, acc);
      keyTimestampMap_[acc_key] = timestamp;
    }
    
    appPtr_->getLogger().debug("GraphTimeCentricKimera: Created initial values for state " + 
                               std::to_string(state_idx) + " at timestamp " + 
                               std::to_string(timestamp));
    
    return true;
    
  } catch (const gtsam::ValuesKeyAlreadyExists& e) {
    appPtr_->getLogger().error("GraphTimeCentricKimera: Key already exists when creating state " + 
                               std::to_string(state_idx) + ": " + std::string(e.what()));
    return false;
    
  } catch (const std::exception& e) {
    appPtr_->getLogger().error("GraphTimeCentricKimera: Exception creating initial values: " + 
                               std::string(e.what()));
    return false;
  }
}

size_t GraphTimeCentricKimera::findNearestState(double timestamp, double tolerance) const {
  size_t nearest_idx = 0;
  double min_diff = std::numeric_limits<double>::max();
  
  // Search in currentKeyIndexTimestampMap_
  for (const auto& [idx, ts] : currentKeyIndexTimestampMap_) {
    double diff = std::abs(ts - timestamp);
    
    if (diff < tolerance && diff < min_diff) {
      min_diff = diff;
      nearest_idx = idx;
    }
  }
  
  return nearest_idx;
}

bool GraphTimeCentricKimera::checkLandmarkCheirality(const LandmarkId& lm_id, 
                                                      const gtsam::Values& values) const {
  // TODO: Implement in Phase 3
  return true;
}

GraphTimeCentricKimera::SmartFactorPtr GraphTimeCentricKimera::createSmartFactor(
    const FeatureTrack& track) const {
  // TODO: Implement in Phase 3
  return nullptr;
}

} // namespace fgo::graph
