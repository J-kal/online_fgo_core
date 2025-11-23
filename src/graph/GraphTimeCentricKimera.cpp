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
#include <gtsam/inference/Symbol.h>
#include <gtsam/inference/Key.h>
#include <algorithm>
#include <cmath>
#include <sstream>

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
    
    // Load initialization parameters from BackendParams.yaml (matching Kimera-VIO)
    kimeraParams_.initialPositionSigma = params.getDouble("initialPositionSigma", 0.00001);
    kimeraParams_.initialRollPitchSigma = params.getDouble("initialRollPitchSigma", 10.0 / 180.0 * M_PI);
    kimeraParams_.initialYawSigma = params.getDouble("initialYawSigma", 0.1 / 180.0 * M_PI);
    kimeraParams_.initialVelocitySigma = params.getDouble("initialVelocitySigma", 1e-3);
    kimeraParams_.initialAccBiasSigma = params.getDouble("initialAccBiasSigma", 0.1);
    kimeraParams_.initialGyroBiasSigma = params.getDouble("initialGyroBiasSigma", 0.01);
    
    appPtr_->getLogger().info("GraphTimeCentricKimera: Kimera support initialized successfully");
    appPtr_->getLogger().info("  - Timestamp tolerance: " + std::to_string(kimeraParams_.timestampMatchTolerance) + " s");
    appPtr_->getLogger().info("  - Use combined IMU factor: " + std::string(kimeraParams_.useCombinedIMUFactor ? "true" : "false"));
    // appPtr_->getLogger().info("  - Add GP motion priors: " + std::string(kimeraParams_.addGPMotionPriors ? "true" : "false"));
    appPtr_->getLogger().info("  - Add GP motion priors: DISABLED (commented out for IMU isolation testing)");
    appPtr_->getLogger().info("  - Initial position sigma: " + std::to_string(kimeraParams_.initialPositionSigma) + " m");
    appPtr_->getLogger().info("  - Initial roll/pitch sigma: " + std::to_string(kimeraParams_.initialRollPitchSigma) + " rad");
    appPtr_->getLogger().info("  - Initial yaw sigma: " + std::to_string(kimeraParams_.initialYawSigma) + " rad");
    appPtr_->getLogger().info("  - Initial velocity sigma: " + std::to_string(kimeraParams_.initialVelocitySigma) + " m/s");
    appPtr_->getLogger().info("  - Initial acc bias sigma: " + std::to_string(kimeraParams_.initialAccBiasSigma) + " m/s²");
    appPtr_->getLogger().info("  - Initial gyro bias sigma: " + std::to_string(kimeraParams_.initialGyroBiasSigma) + " rad/s");
    
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


StatusGraphConstruction GraphTimeCentricKimera::constructFactorGraphFromTimestamps(
    const std::vector<double>& timestamps) {
  
  if (timestamps.empty()) {
    appPtr_->getLogger().error("GraphTimeCentricKimera: No timestamps provided for graph construction");
    return StatusGraphConstruction::FAILED;
  }

  if (timestamps.size() < 2) {
    appPtr_->getLogger().warn("GraphTimeCentricKimera: Not enough timestamps to construct a factor graph. Need at least 2.");
    return StatusGraphConstruction::NO_OPTIMIZATION;
  }
  
  std::cout << "[online_fgo_core] GraphTimeCentricKimera: CONSTRUCT FACTOR GRAPH FROM " << timestamps.size() << " TIMESTAMPS" << std::endl;
  
  appPtr_->getLogger().info("GraphTimeCentricKimera: Constructing factor graph from " + 
                            std::to_string(timestamps.size()) + " timestamps");
  
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
  
  // Create IMU factors between consecutive states using stored PIM values
  // PIM[i] corresponds to preintegration from state[i-1] to state[i]
  // stored_pims_[i] contains PIM for transition to timestamps[i]
  for (size_t i = 1; i < created_states.size(); ++i) {
    size_t state_i = created_states[i-1];
    size_t state_j = created_states[i];
    
    // Match PIM by index: stored_pims_[i] contains PIM for transition to timestamps[i]
    // For state transition from created_states[i-1] to created_states[i]:
    // - We need PIM stored with timestamp of created_states[i] (destination)
    // - stored_pims_ is indexed by destination timestamp
    double ts_j = keyTimestampMap_[X(state_j)];
    std::shared_ptr<gtsam::PreintegrationType> pim_for_pair = nullptr;
    
    // Find PIM matching destination timestamp
    for (const auto& pim_pair : stored_pims_) {
      if (std::abs(pim_pair.first - ts_j) < kimeraParams_.timestampMatchTolerance) {
        pim_for_pair = pim_pair.second;
        break;
      }
    }
    
    if (pim_for_pair) {
      // Use stored PIM directly (efficient, no re-preintegration)
      if (!addIMUFactorFromPIM(state_i, state_j, *pim_for_pair)) {
        appPtr_->getLogger().error("GraphTimeCentricKimera: Failed to add IMU factor from PIM between states " +
                                   std::to_string(state_i) + " and " + std::to_string(state_j));
        return StatusGraphConstruction::FAILED;
      }
    } else {
      appPtr_->getLogger().warn("GraphTimeCentricKimera: No PIM available for keyframe pair " +
                                std::to_string(state_i) + " -> " + std::to_string(state_j) +
                                " at timestamp " + std::to_string(ts_j));
      // Continue without IMU factor for this pair
      continue;
    }

    // GP MOTION PRIORS - COMMENTED OUT FOR IMU ISOLATION TESTING
    /*
    // Also add GP motion prior if enabled
    if (kimeraParams_.addGPMotionPriors) {
      if (!addGPMotionPriorBetweenStates(state_i, state_j)) {
        appPtr_->getLogger().error("GraphTimeCentricKimera: Failed to add GP motion prior between states " +
                                   std::to_string(state_i) + " and " + std::to_string(state_j));
        return StatusGraphConstruction::FAILED;
      }
    }
    */
  }

  appPtr_->getLogger().info("GraphTimeCentricKimera: Factor graph construction successful");
  return StatusGraphConstruction::SUCCESSFUL;
}

bool GraphTimeCentricKimera::addIMUFactorFromPIM(
    size_t state_i_idx,
    size_t state_j_idx,
    const gtsam::PreintegrationType& pim) {
  
  try {
    // Get keys for states
    gtsam::Key pose_i = X(state_i_idx);
    gtsam::Key vel_i = V(state_i_idx);
    gtsam::Key bias_i = B(state_i_idx);
    gtsam::Key pose_j = X(state_j_idx);
    gtsam::Key vel_j = V(state_j_idx);
    gtsam::Key bias_j = B(state_j_idx);
    
    // Create CombinedImuFactor from PIM (only CombinedMeasurements supported)
    const auto* combined_pim = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements*>(&pim);
    if (!combined_pim) {
      appPtr_->getLogger().error("GraphTimeCentricKimera: PIM must be PreintegratedCombinedMeasurements");
      return false;
    }
    
    boost::shared_ptr<gtsam::CombinedImuFactor> imu_factor =
        boost::make_shared<gtsam::CombinedImuFactor>(
            pose_i, vel_i,
            pose_j, vel_j,
            bias_i, bias_j,
            *combined_pim);
    
    this->push_back(imu_factor);
    
    appPtr_->getLogger().debug("GraphTimeCentricKimera: Added CombinedImuFactor from PIM between states " + 
                               std::to_string(state_i_idx) + " and " + 
                               std::to_string(state_j_idx));
    
    return true;
    
  } catch (const std::exception& e) {
    appPtr_->getLogger().error("GraphTimeCentricKimera: Exception while adding IMU factor from PIM: " + 
                               std::string(e.what()));
    return false;
  }
}

bool GraphTimeCentricKimera::addPreintegratedIMUData(
    const std::vector<std::pair<double, std::shared_ptr<gtsam::PreintegrationType>>>& pim_data) {
  
  if (pim_data.empty()) {
    appPtr_->getLogger().warn("GraphTimeCentricKimera: Empty PIM data provided");
    return false;
  }
  
  std::cout << "[online_fgo_core] GraphTimeCentricKimera: ADDING " << pim_data.size() 
            << " PREINTEGRATED IMU MEASUREMENTS" << std::endl;
  
  appPtr_->getLogger().info("GraphTimeCentricKimera: Storing " + 
                            std::to_string(pim_data.size()) + " preintegrated IMU measurements");
  
  stored_pims_.clear();
  appPtr_->getLogger().debug("GraphTimeCentricKimera: Cleared previous PIM values");
  
  // Store PIM values for later use in factor construction
  stored_pims_.insert(stored_pims_.end(), pim_data.begin(), pim_data.end());
  
  appPtr_->getLogger().debug("GraphTimeCentricKimera: Successfully stored " + 
                             std::to_string(pim_data.size()) + " PIM values");
  
  return true;
}

// ========================================================================
// GP MOTION PRIORS - COMMENTED OUT FOR IMU ISOLATION TESTING
// ========================================================================

/*
bool GraphTimeCentricKimera::addGPMotionPriorsForStates(const std::vector<size_t>& state_indices) {
  if (state_indices.size() < 2) {
    appPtr_->getLogger().warn("GraphTimeCentricKimera: Need at least 2 states for GP priors");
    return true;
  }

  appPtr_->getLogger().info("GraphTimeCentricKimera: Adding GP motion priors for " +
                            std::to_string(state_indices.size() - 1) + " state pairs");

  for (size_t i = 1; i < state_indices.size(); ++i) {
    if (!addGPMotionPriorBetweenStates(state_indices[i-1], state_indices[i])) {
      return false;
    }
  }

  appPtr_->getLogger().info("GraphTimeCentricKimera: Successfully added " +
                            std::to_string(state_indices.size() - 1) + " GP motion priors");

  return true;
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
*/

// Stub implementations to satisfy interface (return true, do nothing)
bool GraphTimeCentricKimera::addGPMotionPriorsForStates(const std::vector<size_t>& /*state_indices*/) {
  // GP priors disabled for IMU isolation testing
  return true;
}

bool GraphTimeCentricKimera::addGPMotionPriorBetweenStates(size_t /*state_i_idx*/, size_t /*state_j_idx*/) {
  // GP priors disabled for IMU isolation testing
  return true;
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
  
  // Validate all factor keys exist in values_ before optimization
  // This prevents "invalid key" errors from the solver
  bool all_keys_exist = true;
  std::vector<gtsam::Key> missing_keys;
  
  for (const auto& factor : *this) {
    if (factor) {
      for (const auto& key : factor->keys()) {
        if (!values_.exists(key)) {
          all_keys_exist = false;
          missing_keys.push_back(key);
        }
      }
    }
  }
  
  if (!all_keys_exist) {
    std::ostringstream error_msg;
    error_msg << "GraphTimeCentricKimera: Missing keys in values_ before optimization: ";
    for (size_t i = 0; i < missing_keys.size() && i < 10; ++i) {
      error_msg << gtsam::DefaultKeyFormatter(missing_keys[i]) << " ";
    }
    if (missing_keys.size() > 10) {
      error_msg << "... (and " << (missing_keys.size() - 10) << " more)";
    }
    appPtr_->getLogger().error(error_msg.str());
    
    // Try to add missing keys with default values (this should not happen in normal operation)
    for (const auto& key : missing_keys) {
      gtsam::Symbol symbol(key);
      if (symbol.chr() == 'x') {
        values_.insert(key, gtsam::Pose3());
        appPtr_->getLogger().warn("GraphTimeCentricKimera: Added missing pose key " + 
                                  gtsam::DefaultKeyFormatter(key) + " with default value");
      } else if (symbol.chr() == 'v') {
        // Create actual Vector3 object (not expression type - Vector3::Zero() returns expression)
        // Use default constructor which initializes to zero
        gtsam::Vector3 zero_vel(0.0, 0.0, 0.0);
        values_.insert(key, zero_vel);
        appPtr_->getLogger().warn("GraphTimeCentricKimera: Added missing velocity key " + 
                                  gtsam::DefaultKeyFormatter(key) + " with default value");
      } else if (symbol.chr() == 'b') {
        values_.insert(key, gtsam::imuBias::ConstantBias());
        appPtr_->getLogger().warn("GraphTimeCentricKimera: Added missing bias key " + 
                                  gtsam::DefaultKeyFormatter(key) + " with default value");
      }
    }
  }
  
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
    
    // Insert or update initial values (handle existing keys gracefully)
    if (values_.exists(pose_key)) {
      values_.update(pose_key, predicted_state.state.pose());
    } else {
      values_.insert(pose_key, predicted_state.state.pose());
    }
    
    if (values_.exists(vel_key)) {
      values_.update(vel_key, predicted_state.state.velocity());
    } else {
      values_.insert(vel_key, predicted_state.state.velocity());
    }
    
    if (values_.exists(bias_key)) {
      values_.update(bias_key, predicted_state.imuBias);
    } else {
      values_.insert(bias_key, predicted_state.imuBias);
    }
    
    // Update keyTimestampMap
    keyTimestampMap_[pose_key] = timestamp;
    keyTimestampMap_[vel_key] = timestamp;
    keyTimestampMap_[bias_key] = timestamp;
    
    // Update currentKeyIndexTimestampMap_
    currentKeyIndexTimestampMap_.insert(std::make_pair(state_idx, timestamp));
    
        // GP FACTORS - COMMENTED OUT FOR IMU ISOLATION TESTING
        /*
        // If GP factors enabled, also insert omega and acc keys
        if (kimeraParams_.addGPMotionPriors) {
          gtsam::Key omega_key = W(state_idx);
          values_.insert(omega_key, predicted_state.omega);
          keyTimestampMap_[omega_key] = timestamp;
          
          gtsam::Key acc_key = A(state_idx);
          // Get acceleration from buffer if available, otherwise use zero
          gtsam::Vector6 acc = gtsam::Vector6::Zero();
          if (state_idx > 0 && accBuffer_.size() > (state_idx - 1)) {
            acc = accBuffer_.get_buffer_from_id(state_idx - 1);
          }
          values_.insert(acc_key, acc);
          keyTimestampMap_[acc_key] = timestamp;
        }
        */    
    appPtr_->getLogger().debug("GraphTimeCentricKimera: Created initial values for state " + 
                               std::to_string(state_idx) + " at timestamp " + 
                               std::to_string(timestamp));
    
    return true;
    
  } catch (const std::exception& e) {
    appPtr_->getLogger().error("GraphTimeCentricKimera: Exception creating initial values: " + 
                               std::string(e.what()));
    return false;
  }
}

bool GraphTimeCentricKimera::setStateInitialValues(size_t state_idx,
                                                    const gtsam::Pose3& pose,
                                                    const gtsam::Vector3& velocity,
                                                    const gtsam::imuBias::ConstantBias& bias) {
  try {
    // Get keys for the state
    gtsam::Key pose_key = X(state_idx);
    gtsam::Key vel_key = V(state_idx);
    gtsam::Key bias_key = B(state_idx);
    
    // Update or insert initial values
    if (values_.exists(pose_key)) {
      values_.update(pose_key, pose);
    } else {
      values_.insert(pose_key, pose);
    }
    
    if (values_.exists(vel_key)) {
      values_.update(vel_key, velocity);
    } else {
      values_.insert(vel_key, velocity);
    }
    
    if (values_.exists(bias_key)) {
      values_.update(bias_key, bias);
    } else {
      values_.insert(bias_key, bias);
    }
    
    // Add prior factors to the first state if not already added
    // This ensures the system is well-constrained (matching Kimera-VIO approach)
    if (state_idx == 1 && !first_state_priors_added_) {
      double timestamp = keyTimestampMap_[pose_key];
      if (addPriorFactorsToFirstState(state_idx, timestamp)) {
        first_state_priors_added_ = true;
        appPtr_->getLogger().info("GraphTimeCentricKimera: Added prior factors to first state after setting initial values");
      } else {
        appPtr_->getLogger().warn("GraphTimeCentricKimera: Failed to add prior factors to first state");
      }
    }
    
    appPtr_->getLogger().debug("GraphTimeCentricKimera: Set initial values for state " + 
                               std::to_string(state_idx) + 
                               " - pose: [" + std::to_string(pose.translation().x()) + ", " +
                               std::to_string(pose.translation().y()) + ", " +
                               std::to_string(pose.translation().z()) + "], " +
                               "velocity: [" + std::to_string(velocity.x()) + ", " +
                               std::to_string(velocity.y()) + ", " +
                               std::to_string(velocity.z()) + "]");
    
    return true;
    
  } catch (const std::exception& e) {
    appPtr_->getLogger().error("GraphTimeCentricKimera: Exception setting initial values: " + 
                               std::string(e.what()));
    return false;
  }
}

bool GraphTimeCentricKimera::addPriorFactorsToFirstState(size_t state_idx, double timestamp) {
  try {
    // Get the current values for the state
    gtsam::Key pose_key = X(state_idx);
    gtsam::Key vel_key = V(state_idx);
    gtsam::Key bias_key = B(state_idx);
    
    if (!values_.exists(pose_key) || !values_.exists(vel_key) || !values_.exists(bias_key)) {
      appPtr_->getLogger().error("GraphTimeCentricKimera: Cannot add priors - values not found for state " + 
                                 std::to_string(state_idx));
      return false;
    }
    
    // Get the values
    gtsam::Pose3 pose = values_.at<gtsam::Pose3>(pose_key);
    gtsam::Vector3 velocity = values_.at<gtsam::Vector3>(vel_key);
    gtsam::imuBias::ConstantBias bias = values_.at<gtsam::imuBias::ConstantBias>(bias_key);
    
    // Create pose prior covariance matching Kimera-VIO's approach
    // Set initial pose uncertainty: constrain mainly position and global yaw.
    // Roll and pitch are observable, therefore low variance.
    gtsam::Matrix6 pose_prior_covariance = gtsam::Matrix6::Zero();
    pose_prior_covariance.diagonal()[0] = kimeraParams_.initialRollPitchSigma * kimeraParams_.initialRollPitchSigma;
    pose_prior_covariance.diagonal()[1] = kimeraParams_.initialRollPitchSigma * kimeraParams_.initialRollPitchSigma;
    pose_prior_covariance.diagonal()[2] = kimeraParams_.initialYawSigma * kimeraParams_.initialYawSigma;
    pose_prior_covariance.diagonal()[3] = kimeraParams_.initialPositionSigma * kimeraParams_.initialPositionSigma;
    pose_prior_covariance.diagonal()[4] = kimeraParams_.initialPositionSigma * kimeraParams_.initialPositionSigma;
    pose_prior_covariance.diagonal()[5] = kimeraParams_.initialPositionSigma * kimeraParams_.initialPositionSigma;
    
    // Rotate initial uncertainty into local frame, where the uncertainty is specified.
    // This matches Kimera-VIO's approach: rotate the rotation part of the covariance
    gtsam::Matrix3 B_Rot_W = pose.rotation().matrix().transpose();
    pose_prior_covariance.topLeftCorner(3, 3) = 
        B_Rot_W * pose_prior_covariance.topLeftCorner(3, 3) * B_Rot_W.transpose();
    
    // Create noise models using loaded parameters
    gtsam::SharedNoiseModel noise_init_pose = 
        gtsam::noiseModel::Gaussian::Covariance(pose_prior_covariance);
    
    gtsam::SharedNoiseModel noise_init_vel_prior = 
        gtsam::noiseModel::Isotropic::Sigma(3, kimeraParams_.initialVelocitySigma);
    
    // Bias prior noise model
    gtsam::Vector6 prior_bias_sigmas;
    prior_bias_sigmas.head<3>().setConstant(kimeraParams_.initialAccBiasSigma);
    prior_bias_sigmas.tail<3>().setConstant(kimeraParams_.initialGyroBiasSigma);
    gtsam::SharedNoiseModel imu_bias_prior_noise = 
        gtsam::noiseModel::Diagonal::Sigmas(prior_bias_sigmas);
    
    // Create and add prior factors (matching Kimera-VIO's addInitialPriorFactors)
    auto prior_pose = gtsam::PriorFactor<gtsam::Pose3>(pose_key, pose, noise_init_pose);
    this->push_back(prior_pose);
    
    auto prior_vel = gtsam::PriorFactor<gtsam::Vector3>(vel_key, velocity, noise_init_vel_prior);
    this->push_back(prior_vel);
    
    auto prior_bias = gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(bias_key, bias, imu_bias_prior_noise);
    this->push_back(prior_bias);
    
    appPtr_->getLogger().info("GraphTimeCentricKimera: Added prior factors to first state " + 
                              std::to_string(state_idx) + " at timestamp " + 
                              std::to_string(timestamp) + 
                              " (using parameters from BackendParams.yaml)");
    
    return true;
    
  } catch (const std::exception& e) {
    appPtr_->getLogger().error("GraphTimeCentricKimera: Exception adding prior factors: " + 
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

