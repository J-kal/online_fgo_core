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
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
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
    
    // NOTE: IMU factor configuration (imuPreintegrationType, accRandomWalk, gyroRandomWalk, 
    // nominalSamplingTimeS) is set via setKimeraParams() from Kimera's ImuParams.yaml,
    // NOT from this params file. Do not override them here.
    
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
    appPtr_->getLogger().info("  - IMU preintegration type: " + 
        std::to_string(static_cast<int>(kimeraParams_.imuPreintegrationType)) + 
        " (0=Combined, 1=Regular+BiasFactor)");
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

bool GraphTimeCentricKimera::addKeyframeState(double timestamp,
                                              size_t frame_id,
                                              const gtsam::Pose3& pose,
                                              const gtsam::Vector3& velocity,
                                              const gtsam::imuBias::ConstantBias& bias,
                                              size_t& out_state_idx) {
  // Use Kimera's frame_id as the state index to ensure key alignment with VioBackend
  // This ensures that X(frame_id), V(frame_id), B(frame_id) keys match what VioBackend expects
  size_t state_idx = frame_id;
  
  // Check if state already exists at this index
  gtsam::Key pose_key = X(state_idx);
  if (values_.exists(pose_key)) {
    appPtr_->getLogger().debug("GraphTimeCentricKimera: State " + std::to_string(state_idx) + 
                               " already exists, updating values");
  } else {
    // Create initial values for the new state
    if (!createInitialValuesForState(state_idx, timestamp)) {
      appPtr_->getLogger().error("GraphTimeCentricKimera: Failed to create initial values for state " +
                                 std::to_string(state_idx));
      return false;
    }
    // Update timestamp mapping
    updateTimestampIndex(timestamp, state_idx);
  }
  
  // Track the highest state index for nState_
  if (state_idx > nState_) {
    nState_ = state_idx;
  }
  
  appPtr_->getLogger().info("GraphTimeCentricKimera: Creating new state " + 
                            std::to_string(state_idx) + " at timestamp " + 
                            std::to_string(timestamp));
  
  bool initial_values_set = setStateInitialValues(state_idx, pose, velocity, bias);
  if (!initial_values_set) {
    appPtr_->getLogger().error("GraphTimeCentricKimera: Failed to set initial values for state " +
                              std::to_string(state_idx));
    return false;
  }
  appPtr_->getLogger().info("GraphTimeCentricKimera: Initial values set for state " +
                            std::to_string(state_idx));
  out_state_idx = state_idx;
  return true;
}

bool GraphTimeCentricKimera::bootstrapInitialState(double timestamp,
                                                   size_t frame_id,
                                                   const gtsam::Pose3& pose,
                                                   const gtsam::Vector3& velocity,
                                                   const gtsam::imuBias::ConstantBias& bias,
                                                   size_t& out_state_idx) {
  if (first_state_priors_added_) {
    appPtr_->getLogger().warn("GraphTimeCentricKimera: bootstrapInitialState called twice");
    return false;
  }

  // Use Kimera's frame_id as the state index to ensure key alignment with VioBackend
  size_t state_idx = frame_id;
  
  // Create initial values for the state
  if (!createInitialValuesForState(state_idx, timestamp)) {
    appPtr_->getLogger().error("GraphTimeCentricKimera: failed to create initial state");
    return false;
  }
  
  // Update timestamp mapping
  updateTimestampIndex(timestamp, state_idx);
  
  // Track the highest state index for nState_
  if (state_idx > nState_) {
    nState_ = state_idx;
  }
  
  bool initial_values_set = setStateInitialValues(state_idx, pose, velocity, bias);
  if (!initial_values_set) {
    appPtr_->getLogger().error("GraphTimeCentricKimera: failed to set initial values during bootstrap");
    return false;
  }

  bool priors_added = addPriorFactorsToFirstState(state_idx, timestamp);
  if (!priors_added) {
    appPtr_->getLogger().error("GraphTimeCentricKimera: failed to add priors during bootstrap");
    return false;
  }

  first_state_priors_added_ = true;
  appPtr_->getLogger().info("GraphTimeCentricKimera: bootstrap initial state complete");
  out_state_idx = state_idx;
  return true;
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
  
  // Get keys for states (matches VioBackend::addImuFactor structure)
  gtsam::Key pose_i = X(state_i_idx);
  gtsam::Key vel_i = V(state_i_idx);
  gtsam::Key bias_i = B(state_i_idx);
  gtsam::Key pose_j = X(state_j_idx);
  gtsam::Key vel_j = V(state_j_idx);
  gtsam::Key bias_j = B(state_j_idx);
  
  // Switch on IMU preintegration type (mirrors VioBackend::addImuFactor exactly)
  switch (kimeraParams_.imuPreintegrationType) {
    case ImuPreintegrationType::kPreintegratedCombinedMeasurements: {
      // CombinedImuFactor includes bias evolution internally
      const auto* combined_pim = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements*>(&pim);
      if (!combined_pim) {
        appPtr_->getLogger().error("GraphTimeCentricKimera: Expected PreintegratedCombinedMeasurements but got different type");
        return false;
      }
      
      auto imu_factor = boost::make_shared<gtsam::CombinedImuFactor>(
          pose_i, vel_i,
          pose_j, vel_j,
          bias_i, bias_j,
          *combined_pim);
      
      this->push_back(imu_factor);
      new_factors_since_last_opt_.push_back(imu_factor);
      
      appPtr_->getLogger().debug("GraphTimeCentricKimera: Added CombinedImuFactor between states " + 
                                 std::to_string(state_i_idx) + " and " + std::to_string(state_j_idx));
      break;
    }
    case ImuPreintegrationType::kPreintegratedImuMeasurements: {
      // ImuFactor + separate BetweenFactor for bias evolution (matches VioBackend exactly)
      const auto* regular_pim = dynamic_cast<const gtsam::PreintegratedImuMeasurements*>(&pim);
      if (!regular_pim) {
        appPtr_->getLogger().error("GraphTimeCentricKimera: Expected PreintegratedImuMeasurements but got different type");
        return false;
      }
      
      // Create ImuFactor (uses only bias_i, not bias_j)
      auto imu_factor = boost::make_shared<gtsam::ImuFactor>(
          pose_i, vel_i,
          pose_j, vel_j,
          bias_i,
          *regular_pim);
      
      this->push_back(imu_factor);
      new_factors_since_last_opt_.push_back(imu_factor);
      
      // Add bias evolution factor (matches VioBackend::addImuFactor exactly)
      // See Trawny05 http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf Eq. 130
      static const gtsam::imuBias::ConstantBias zero_bias(
          gtsam::Vector3(0.0, 0.0, 0.0), gtsam::Vector3(0.0, 0.0, 0.0));
      
      if (kimeraParams_.nominalSamplingTimeS == 0.0) {
        appPtr_->getLogger().error("GraphTimeCentricKimera: nominalSamplingTimeS cannot be 0");
        return false;
      }
      
      const double sqrt_delta_t_ij = std::sqrt(pim.deltaTij());
      gtsam::Vector6 bias_sigmas;
      bias_sigmas.head<3>().setConstant(sqrt_delta_t_ij * kimeraParams_.accRandomWalk);
      bias_sigmas.tail<3>().setConstant(sqrt_delta_t_ij * kimeraParams_.gyroRandomWalk);
      
      const gtsam::SharedNoiseModel bias_noise_model =
          gtsam::noiseModel::Diagonal::Sigmas(bias_sigmas);
      
      auto bias_between_factor =
          boost::make_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
              bias_i, bias_j, zero_bias, bias_noise_model);
      
      this->push_back(bias_between_factor);
      new_factors_since_last_opt_.push_back(bias_between_factor);
      
      appPtr_->getLogger().debug("GraphTimeCentricKimera: Added ImuFactor + BiasBetweenFactor between states " + 
                                 std::to_string(state_i_idx) + " and " + std::to_string(state_j_idx));
      break;
    }
    default: {
      appPtr_->getLogger().error("GraphTimeCentricKimera: Unknown IMU Preintegration Type");
      return false;
    }
  }
  
  return true;
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

bool GraphTimeCentricKimera::buildIncrementalUpdate(
    gtsam::NonlinearFactorGraph* new_factors,
    gtsam::Values* new_values,
    fgo::solvers::FixedLagSmoother::KeyTimestampMap* new_timestamps) {
  
  if (!new_factors || !new_values || !new_timestamps) {
    appPtr_->getLogger().warn(
        "GraphTimeCentricKimera::buildIncrementalUpdate: null output pointers: "
        "new_factors=" + std::to_string(new_factors != nullptr) +
        " new_values=" + std::to_string(new_values != nullptr) +
        " new_timestamps=" + std::to_string(new_timestamps != nullptr));
    return false;
  }

  new_factors->resize(0);
  new_values->clear();
  new_timestamps->clear();

  // Debug current graph / value state before deciding what to return.
  appPtr_->getLogger().info(
      "GraphTimeCentricKimera::buildIncrementalUpdate: "
      "graph_size=" + std::to_string(this->size()) +
      " values_size=" + std::to_string(values_.size()) +
      " keyTimestampMap_size=" + std::to_string(keyTimestampMap_.size()) +
      " first_state_priors_added_=" + std::to_string(first_state_priors_added_));

  if (this->size() == 0) {
    appPtr_->getLogger().warn(
        "GraphTimeCentricKimera::buildIncrementalUpdate: no factors in graph, returning false");
    return false;
  }

  if (new_factors_since_last_opt_.empty() && new_values_since_last_opt_.empty()) {
    appPtr_->getLogger().info(
        "GraphTimeCentricKimera::buildIncrementalUpdate: no new factors/values since last opt");
    return false;
  }

  *new_factors = new_factors_since_last_opt_;
  *new_values = new_values_since_last_opt_;

  for (const auto& [key, timestamp] : new_key_timestamps_since_last_opt_) {
    (*new_timestamps)[key] = timestamp;
  }

  return true;
}

void GraphTimeCentricKimera::finalizeIncrementalUpdate() {
  new_factors_since_last_opt_.resize(0);
  new_values_since_last_opt_.clear();
  new_key_timestamps_since_last_opt_.clear();
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
    auto buffer_pair = currentPredictedBuffer_.get_all_time_buffer_pair();
    appPtr_->getLogger().debug(
        "GraphTimeCentricKimera::createInitialValuesForState: state_idx=" +
        std::to_string(state_idx) + " ts=" + std::to_string(timestamp) +
        " currentPredictedBuffer_size=" +
        std::to_string(buffer_pair.size()));

    // Query predicted state from currentPredictedBuffer_
    auto predicted_state = graph::queryCurrentPredictedState(
        buffer_pair, timestamp);
    
    // Create keys
    gtsam::Key pose_key = X(state_idx);
    gtsam::Key vel_key = V(state_idx);
    gtsam::Key bias_key = B(state_idx);
    
    // Insert or update initial values (handle existing keys gracefully)
    if (values_.exists(pose_key)) {
      values_.update(pose_key, predicted_state.state.pose());
    } else {
      values_.insert(pose_key, predicted_state.state.pose());
      new_values_since_last_opt_.insert(pose_key, predicted_state.state.pose());
      new_key_timestamps_since_last_opt_[pose_key] = timestamp;
    }
    
    if (values_.exists(vel_key)) {
      values_.update(vel_key, predicted_state.state.velocity());
    } else {
      values_.insert(vel_key, predicted_state.state.velocity());
      new_values_since_last_opt_.insert(vel_key, predicted_state.state.velocity());
      new_key_timestamps_since_last_opt_[vel_key] = timestamp;
    }
    
    if (values_.exists(bias_key)) {
      values_.update(bias_key, predicted_state.imuBias);
    } else {
      values_.insert(bias_key, predicted_state.imuBias);
      new_values_since_last_opt_.insert(bias_key, predicted_state.imuBias);
      new_key_timestamps_since_last_opt_[bias_key] = timestamp;
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
    appPtr_->getLogger().debug(
        "GraphTimeCentricKimera::setStateInitialValues: state_idx=" +
        std::to_string(state_idx));

    // Get keys for the state
    gtsam::Key pose_key = X(state_idx);
    gtsam::Key vel_key = V(state_idx);
    gtsam::Key bias_key = B(state_idx);
    
    // Update or insert initial values
    if (values_.exists(pose_key)) {
      values_.update(pose_key, pose);
    } else {
      values_.insert(pose_key, pose);
      new_values_since_last_opt_.insert(pose_key, pose);
      if (keyTimestampMap_.find(pose_key) != keyTimestampMap_.end()) {
        new_key_timestamps_since_last_opt_[pose_key] = keyTimestampMap_[pose_key];
      }
    }
    
    if (values_.exists(vel_key)) {
      values_.update(vel_key, velocity);
    } else {
      values_.insert(vel_key, velocity);
      new_values_since_last_opt_.insert(vel_key, velocity);
      if (keyTimestampMap_.find(vel_key) != keyTimestampMap_.end()) {
        new_key_timestamps_since_last_opt_[vel_key] = keyTimestampMap_[vel_key];
      }
    }
    
    if (values_.exists(bias_key)) {
      values_.update(bias_key, bias);
    } else {
      values_.insert(bias_key, bias);
      new_values_since_last_opt_.insert(bias_key, bias);
      if (keyTimestampMap_.find(bias_key) != keyTimestampMap_.end()) {
        new_key_timestamps_since_last_opt_[bias_key] = keyTimestampMap_[bias_key];
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
    appPtr_->getLogger().info(
        "GraphTimeCentricKimera::addPriorFactorsToFirstState: state_idx=" +
        std::to_string(state_idx) + " ts=" + std::to_string(timestamp) +
        " kimera sigmas: pos=" + std::to_string(kimeraParams_.initialPositionSigma) +
        " rollPitch=" + std::to_string(kimeraParams_.initialRollPitchSigma) +
        " yaw=" + std::to_string(kimeraParams_.initialYawSigma) +
        " vel=" + std::to_string(kimeraParams_.initialVelocitySigma) +
        " accBias=" + std::to_string(kimeraParams_.initialAccBiasSigma) +
        " gyroBias=" + std::to_string(kimeraParams_.initialGyroBiasSigma));
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
    new_factors_since_last_opt_.push_back(prior_pose);
    
    auto prior_vel = gtsam::PriorFactor<gtsam::Vector3>(vel_key, velocity, noise_init_vel_prior);
    this->push_back(prior_vel);
    new_factors_since_last_opt_.push_back(prior_vel);
    
    auto prior_bias = gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(bias_key, bias, imu_bias_prior_noise);
    this->push_back(prior_bias);
    new_factors_since_last_opt_.push_back(prior_bias);

    appPtr_->getLogger().info(
        "GraphTimeCentricKimera::addPriorFactorsToFirstState: added 3 priors, "
        "graph_size=" + std::to_string(this->size()) +
        " values_size=" + std::to_string(values_.size()) +
        " keyTimestampMap_size=" + std::to_string(keyTimestampMap_.size()));

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

