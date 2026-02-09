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
#include "online_fgo_core/factor/motion/GPWNOAPrior.h"   // For WNOA GP motion priors
#include "online_fgo_core/factor/motion/GPWNOJPrior.h"   // For WNOJ GP motion priors
#include "online_fgo_core/factor/motion/GPSingerPrior.h" // For Singer GP motion priors
#include "online_fgo_core/integration/KimeraIntegrationInterface.h"  // For OmegaAtState
#include <gtsam/inference/Symbol.h>
#include <gtsam/inference/Key.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
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
    
    // GP prior configuration - DO NOT load here, comes from setKimeraParams()
    // Parameters flow: BackendParams.yaml → adapter.createIntegrationParams() → 
    //                  interface.initialize() → graph.setKimeraParams()
    // Fallback only used if setKimeraParams() was never called (shouldn't happen)
    if (kimeraParams_.addGPMotionPriors) {
      appPtr_->getLogger().warn(
        "GraphTimeCentricKimera: GP priors already enabled from setKimeraParams(). "
        "This is correct - parameters flow from BackendParams.yaml via adapter.");
    }
    
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
    std::string gp_type_str;
    switch (kimeraParams_.gpType) {
      case fgo::data::GPModelType::WNOA: gp_type_str = "WNOA"; break;
      case fgo::data::GPModelType::WNOJ: gp_type_str = "WNOJ"; break;
      case fgo::data::GPModelType::WNOJFull: gp_type_str = "WNOJFull"; break;
      case fgo::data::GPModelType::Singer: gp_type_str = "Singer"; break;
      case fgo::data::GPModelType::SingerFull: gp_type_str = "SingerFull"; break;
      case fgo::data::GPModelType::WNOA_WNOJ: gp_type_str = "WNOA+WNOJ"; break;
      case fgo::data::GPModelType::WNOA_Singer: gp_type_str = "WNOA+Singer"; break;
      case fgo::data::GPModelType::WNOA_WNOJ_Singer: gp_type_str = "WNOA+WNOJ+Singer"; break;
      default: gp_type_str = "Unknown"; break;
    }
    appPtr_->getLogger().info("  - GP motion priors: " + 
        std::string(kimeraParams_.addGPMotionPriors ? "ENABLED" : "DISABLED") +
        (kimeraParams_.addGPMotionPriors ? " (type=" + gp_type_str + ", Qc via setGPPriorParams)" : ""));
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
    // For Kimera integration, skip createInitialValuesForState() since:
    // 1. currentPredictedBuffer_ is not populated (buffer query returns identity pose)
    // 2. VioBackend provides correct PIM-predicted initial values via setStateInitialValues()
    // 3. createInitialValuesForState() would insert wrong values into new_values_since_last_opt_
    //
    // Only create timestamp mappings without querying the buffer
    keyTimestampMap_[pose_key] = timestamp;
    keyTimestampMap_[V(state_idx)] = timestamp;
    keyTimestampMap_[B(state_idx)] = timestamp;
    
    // If GP priors are enabled, we need to track timestamps for omega keys.
    // Acceleration keys are ONLY needed for Full GP variants (WNOJFull, SingerFull)
    // where acceleration is an optimization variable, not for WNOA/WNOJ/Singer
    // where acceleration is computed internally.
    if (kimeraParams_.addGPMotionPriors) {
      keyTimestampMap_[W(state_idx)] = timestamp;
      
      // Only create acceleration keys if using Full GP variants
      if (kimeraParams_.gpType == fgo::data::GPModelType::WNOJFull ||
          kimeraParams_.gpType == fgo::data::GPModelType::SingerFull) {
        keyTimestampMap_[A(state_idx)] = timestamp;
      }
    }

    currentKeyIndexTimestampMap_.insert(std::make_pair(state_idx, timestamp));
    
    // Update timestamp mapping
    updateTimestampIndex(timestamp, state_idx);
    
    appPtr_->getLogger().info("GraphTimeCentricKimera: Skipped buffer query for Kimera state " +
                              std::to_string(state_idx) + " (will use PIM-predicted values)");
  }

  // CRITICAL FIX: Explicitly set the initial values for the new state.
  // Previously, this was skipped assuming VioBackend would call setStateInitialValues separately,
  // but VioBackend does not do that. This ensured keyframe states were never added to new_values.
  setStateInitialValues(state_idx, pose, velocity, bias);

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
      
       // this->push_back(imu_factor);
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
      
       // this->push_back(imu_factor);
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
      
       // this->push_back(bias_between_factor);
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

// Helper method to compute 6D acceleration vectors with proper angular acceleration
std::pair<gtsam::Vector6, gtsam::Vector6> GraphTimeCentricKimera::computeAccelerationVectors(
    const OmegaAtState& omega_i,
    const OmegaAtState& omega_j,
    double dt) const {
  
  // Compute angular acceleration as derivative: (omega_j - omega_i) / dt
  gtsam::Vector3 angular_acc = (omega_j.omega - omega_i.omega) / dt;
  
  // Build 6D acceleration vectors: [angular_acc (3D), linear_acc (3D)]
  // This matches the structure used by convertVwWbToVbWb and GP factors
  gtsam::Vector6 acc_i;
  acc_i << angular_acc, omega_i.acc;
  
  gtsam::Vector6 acc_j;
  acc_j << angular_acc, omega_j.acc;
  
  return {acc_i, acc_j};
}

bool GraphTimeCentricKimera::addGPMotionPrior(
    size_t state_i_idx,
    size_t state_j_idx,
    double dt,
    const OmegaAtState& omega_i,
    const OmegaAtState& omega_j) {
  
  // Check if GP prior params have been initialized (via setGPPriorParams from VioBackend)
  if (!gp_params_initialized_ || !gp_qc_model_) {
    appPtr_->getLogger().warn("GraphTimeCentricKimera::addGPMotionPrior: Qc noise model not initialized " +
                              std::string("- skipping (ensure setGPPriorParams is called)"));
    return false;
  }
  
  // Validate omega data
  if (!omega_i.valid || !omega_j.valid) {
    appPtr_->getLogger().warn("GraphTimeCentricKimera::addGPMotionPrior: Invalid omega for states " +
                              std::to_string(state_i_idx) + " and " + std::to_string(state_j_idx));
    return false;
  }
  
  // Get pose/velocity keys
  gtsam::Key pose_i = X(state_i_idx);
  gtsam::Key vel_i = V(state_i_idx);
  gtsam::Key pose_j = X(state_j_idx);
  gtsam::Key vel_j = V(state_j_idx);
  
  // Create omega keys
  gtsam::Key omega_key_i = W(state_i_idx);
  gtsam::Key omega_key_j = W(state_j_idx);
  
  // Get timestamps for the states
  double timestamp_i = 0.0, timestamp_j = 0.0;
  if (keyTimestampMap_.find(pose_i) != keyTimestampMap_.end()) {
    timestamp_i = keyTimestampMap_[pose_i];
  }
  if (keyTimestampMap_.find(pose_j) != keyTimestampMap_.end()) {
    timestamp_j = keyTimestampMap_[pose_j];
  }
  
  // Add omega_i to values or update
  if (values_.exists(omega_key_i)) {
    values_.update(omega_key_i, omega_i.omega);
  } else {
    values_.insert(omega_key_i, omega_i.omega);
  }
  
  // CRITICAL: Stage for the next optimization to ensure smoother has the latest values
  // For omega keys: they may be updated multiple times before first optimization
  // Always ensure they're in new_values_since_last_opt_ if not yet in smoother
  if (keys_sent_to_smoother_.find(omega_key_i) == keys_sent_to_smoother_.end()) {
    if (new_values_since_last_opt_.exists(omega_key_i)) {
      new_values_since_last_opt_.update(omega_key_i, omega_i.omega);
    } else {
      new_values_since_last_opt_.insert(omega_key_i, omega_i.omega);
    }
    new_key_timestamps_since_last_opt_[omega_key_i] = timestamp_i;
  }
  keyTimestampMap_[omega_key_i] = timestamp_i;
  
  appPtr_->getLogger().debug("GraphTimeCentricKimera: Added/Updated omega key W(" + 
                             std::to_string(state_i_idx) + ") = [" +
                             std::to_string(omega_i.omega.x()) + ", " +
                             std::to_string(omega_i.omega.y()) + ", " +
                             std::to_string(omega_i.omega.z()) + "] rad/s");
  
  // Add omega_j to values or update
  if (values_.exists(omega_key_j)) {
    values_.update(omega_key_j, omega_j.omega);
  } else {
    values_.insert(omega_key_j, omega_j.omega);
  }
  
  // CRITICAL: Stage for the next optimization to ensure smoother has the latest values
  // For omega keys: they may be updated multiple times before first optimization
  // Always ensure they're in new_values_since_last_opt_ if not yet in smoother
  if (keys_sent_to_smoother_.find(omega_key_j) == keys_sent_to_smoother_.end()) {
    if (new_values_since_last_opt_.exists(omega_key_j)) {
      new_values_since_last_opt_.update(omega_key_j, omega_j.omega);
    } else {
      new_values_since_last_opt_.insert(omega_key_j, omega_j.omega);
    }
    new_key_timestamps_since_last_opt_[omega_key_j] = timestamp_j;
  }
  keyTimestampMap_[omega_key_j] = timestamp_j;
  
  appPtr_->getLogger().debug("GraphTimeCentricKimera: Added/Updated omega key W(" + 
                             std::to_string(state_j_idx) + ") = [" +
                             std::to_string(omega_j.omega.x()) + ", " +
                             std::to_string(omega_j.omega.y()) + ", " +
                             std::to_string(omega_j.omega.z()) + "] rad/s");
  
  // CRITICAL FIX: Add STRONG UNARY MEASUREMENT factors on omega variables
  // IMU gyroscope provides DIRECT MEASUREMENTS of bias-corrected angular velocity
  // These are actual sensor observations that strongly constrain omega variables
  // 
  // Noise model calculation:
  // - Gyroscope noise density σ_g ≈ 0.00016968 rad/s/√Hz (from ImuParams.yaml)
  // - At IMU rate f = 200Hz: discrete-time sigma = σ_g * sqrt(f) ≈ 0.0024 rad/s
  // - BUT: This is for individual IMU measurements, not keyframe-boundary values
  // - At keyframe boundaries (5Hz, dt=0.2s): effective sigma = σ_g * sqrt(1/dt) ≈ 0.0038 rad/s
  // 
  // Using conservative sigma = 0.005 rad/s (~0.29°/s) provides strong constraint
  // while accounting for interpolation uncertainty at keyframe boundaries.
  // This is MUCH stronger than typical motion model uncertainty (~0.1 rad/s).
  
  // IMPORTANT: Only add measurement factors when keys are NEW to avoid double-counting
  // Once a key is in the smoother, the IMU factor already constrains its omega via PIM
  
  gtsam::Vector3 omega_measurement_sigmas;
  // Use configurable sigma from kimeraParams_ (loaded from BackendParams.yaml)
  omega_measurement_sigmas << kimeraParams_.omegaMeasurementSigma, 
                              kimeraParams_.omegaMeasurementSigma, 
                              kimeraParams_.omegaMeasurementSigma;
  
  // Add measurement factor for omega_i only if it's a NEW key
  if (keys_sent_to_smoother_.find(omega_key_i) == keys_sent_to_smoother_.end()) {
    auto omega_i_measurement_noise = gtsam::noiseModel::Diagonal::Sigmas(omega_measurement_sigmas);
    auto omega_i_measurement_factor = boost::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(
        omega_key_i, omega_i.omega, omega_i_measurement_noise);
    
    new_factors_since_last_opt_.push_back(omega_i_measurement_factor);
    
    appPtr_->getLogger().debug("GraphTimeCentricKimera: Added omega measurement factor W(" + 
                               std::to_string(state_i_idx) + ") = [" +
                               std::to_string(omega_i.omega.x()) + ", " +
                               std::to_string(omega_i.omega.y()) + ", " +
                               std::to_string(omega_i.omega.z()) + "] ± " +
                               std::to_string(kimeraParams_.omegaMeasurementSigma) + " rad/s (from IMU gyro)");
  }
  
  // Add measurement factor for omega_j only if it's a NEW key
  if (keys_sent_to_smoother_.find(omega_key_j) == keys_sent_to_smoother_.end()) {
    auto omega_j_measurement_noise = gtsam::noiseModel::Diagonal::Sigmas(omega_measurement_sigmas);
    auto omega_j_measurement_factor = boost::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(
        omega_key_j, omega_j.omega, omega_j_measurement_noise);
    
    new_factors_since_last_opt_.push_back(omega_j_measurement_factor);
    
    appPtr_->getLogger().debug("GraphTimeCentricKimera: Added omega measurement factor W(" + 
                               std::to_string(state_j_idx) + ") = [" +
                               std::to_string(omega_j.omega.x()) + ", " +
                               std::to_string(omega_j.omega.y()) + ", " +
                               std::to_string(omega_j.omega.z()) + "] ± " +
                               std::to_string(kimeraParams_.omegaMeasurementSigma) + " rad/s (from IMU gyro)");
  }
  
  // Create GP motion prior based on gpType
  std::string gp_type_name;
  
  switch (kimeraParams_.gpType) {
    case fgo::data::GPModelType::WNOA: {
      auto gp_prior = boost::make_shared<fgo::factor::GPWNOAPrior>(
          pose_i, vel_i, omega_key_i,
          pose_j, vel_j, omega_key_j,
          dt, gp_qc_model_,
          false, true);
      // Only add to new_factors for smoother marginalization - don't accumulate in base graph
      new_factors_since_last_opt_.push_back(gp_prior);
      gp_type_name = "GPWNOAPrior";
      break;
    }
    
    case fgo::data::GPModelType::WNOJ: {
      auto [acc_i, acc_j] = computeAccelerationVectors(omega_i, omega_j, dt);
      
      auto gp_prior = boost::make_shared<fgo::factor::GPWNOJPrior>(
          pose_i, vel_i, omega_key_i,
          pose_j, vel_j, omega_key_j,
          acc_i, acc_j,
          dt, gp_qc_model_,
          false, true);
      // Only add to new_factors for smoother marginalization - don't accumulate in base graph
      new_factors_since_last_opt_.push_back(gp_prior);
      gp_type_name = "GPWNOJPrior";
      break;
    }
    
    case fgo::data::GPModelType::WNOA_WNOJ: {
      auto gp_wnoa = boost::make_shared<fgo::factor::GPWNOAPrior>(
          pose_i, vel_i, omega_key_i,
          pose_j, vel_j, omega_key_j,
          dt, gp_qc_model_,
          false, true);
      // Only add to new_factors for smoother marginalization - don't accumulate in base graph
      new_factors_since_last_opt_.push_back(gp_wnoa);
      
      auto [acc_i, acc_j] = computeAccelerationVectors(omega_i, omega_j, dt);
      
      auto gp_wnoj = boost::make_shared<fgo::factor::GPWNOJPrior>(
          pose_i, vel_i, omega_key_i,
          pose_j, vel_j, omega_key_j,
          acc_i, acc_j,
          dt, gp_qc_model_,
          false, true);
      // Only add to new_factors for smoother marginalization - don't accumulate in base graph
      new_factors_since_last_opt_.push_back(gp_wnoj);
      
      gp_type_name = "GPWNOAPrior + GPWNOJPrior";
      break;
    }
    
    case fgo::data::GPModelType::Singer: {
      auto [acc_i, acc_j] = computeAccelerationVectors(omega_i, omega_j, dt);
      
      auto gp_prior = boost::make_shared<fgo::factor::GPSingerPrior>(
          pose_i, vel_i, omega_key_i,
          pose_j, vel_j, omega_key_j,
          dt, gp_qc_model_, gp_ad_matrix_,
          acc_i, acc_j,
          false, true);
      // Only add to new_factors for smoother marginalization - don't accumulate in base graph
      new_factors_since_last_opt_.push_back(gp_prior);
      gp_type_name = "GPSingerPrior";
      break;
    }
    
    case fgo::data::GPModelType::WNOA_Singer: {
      auto gp_wnoa = boost::make_shared<fgo::factor::GPWNOAPrior>(
          pose_i, vel_i, omega_key_i,
          pose_j, vel_j, omega_key_j,
          dt, gp_qc_model_,
          false, true);
      // Only add to new_factors for smoother marginalization - don't accumulate in base graph
      new_factors_since_last_opt_.push_back(gp_wnoa);
      
      auto [acc_i, acc_j] = computeAccelerationVectors(omega_i, omega_j, dt);
      
      auto gp_singer = boost::make_shared<fgo::factor::GPSingerPrior>(
          pose_i, vel_i, omega_key_i,
          pose_j, vel_j, omega_key_j,
          dt, gp_qc_model_, gp_ad_matrix_,
          acc_i, acc_j,
          false, true);
      // Only add to new_factors for smoother marginalization - don't accumulate in base graph
      new_factors_since_last_opt_.push_back(gp_singer);
      
      gp_type_name = "GPWNOAPrior + GPSingerPrior";
      break;
    }
    
    case fgo::data::GPModelType::WNOA_WNOJ_Singer: {
      auto [acc_i, acc_j] = computeAccelerationVectors(omega_i, omega_j, dt);
      
      auto gp_wnoa = boost::make_shared<fgo::factor::GPWNOAPrior>(
          pose_i, vel_i, omega_key_i,
          pose_j, vel_j, omega_key_j,
          dt, gp_qc_model_,
          false, true);
      // Only add to new_factors for smoother marginalization - don't accumulate in base graph
      new_factors_since_last_opt_.push_back(gp_wnoa);
      
      auto gp_wnoj = boost::make_shared<fgo::factor::GPWNOJPrior>(
          pose_i, vel_i, omega_key_i,
          pose_j, vel_j, omega_key_j,
          acc_i, acc_j,
          dt, gp_qc_model_,
          false, true);
      // Only add to new_factors for smoother marginalization - don't accumulate in base graph
      new_factors_since_last_opt_.push_back(gp_wnoj);
      
      auto gp_singer = boost::make_shared<fgo::factor::GPSingerPrior>(
          pose_i, vel_i, omega_key_i,
          pose_j, vel_j, omega_key_j,
          dt, gp_qc_model_, gp_ad_matrix_,
          acc_i, acc_j,
          false, true);
      // Only add to new_factors for smoother marginalization - don't accumulate in base graph
      new_factors_since_last_opt_.push_back(gp_singer);
      
      gp_type_name = "GPWNOAPrior + GPWNOJPrior + GPSingerPrior";
      break;
    }
    
    case fgo::data::GPModelType::WNOJFull:
    case fgo::data::GPModelType::SingerFull: {
      appPtr_->getLogger().warn("GraphTimeCentricKimera: Full GP variant not implemented, using WNOA");
      auto gp_prior = boost::make_shared<fgo::factor::GPWNOAPrior>(
          pose_i, vel_i, omega_key_i,
          pose_j, vel_j, omega_key_j,
          dt, gp_qc_model_,
          false, true);
      // Only add to new_factors for smoother marginalization - don't accumulate in base graph
      new_factors_since_last_opt_.push_back(gp_prior);
      gp_type_name = "GPWNOAPrior (fallback)";
      break;
    }
      
    default:
      appPtr_->getLogger().error("GraphTimeCentricKimera: Unknown GP model type: " +
                                 std::to_string(static_cast<int>(kimeraParams_.gpType)));
      return false;
  }
  
  appPtr_->getLogger().info("GraphTimeCentricKimera: Added " + gp_type_name + " between states " + 
                            std::to_string(state_i_idx) + " and " + std::to_string(state_j_idx) +
                            " (dt=" + std::to_string(dt) + "s)");
  
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
  // this->push_back(factor); // Disabled for Stateless GTC
  new_factors_since_last_opt_.push_back(factor);
  
  appPtr_->getLogger().debug("GraphTimeCentricKimera: Added external factor connecting " + 
                             std::to_string(state_indices.size()) + " states");
  
  return true;
}

bool GraphTimeCentricKimera::buildIncrementalUpdate(
    gtsam::NonlinearFactorGraph* new_factors,
    gtsam::Values* new_values,
    fgo::solvers::FixedLagSmoother::KeyTimestampMap* new_timestamps,
    const gtsam::NonlinearFactorGraph* smoother_graph,
    gtsam::FactorIndices* delete_slots,
    std::vector<LandmarkId>* new_smart_factor_lmk_ids) {
  
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
  
  // Clear optional outputs if provided
  if (delete_slots) {
    delete_slots->clear();
  }
  if (new_smart_factor_lmk_ids) {
    new_smart_factor_lmk_ids->clear();
  }

  // Debug current graph / value state before deciding what to return.
  appPtr_->getLogger().info(
      "GraphTimeCentricKimera::buildIncrementalUpdate: "
      "GraphSize=" + std::to_string(this->size()) +
      ", new_factors=" + std::to_string(new_factors_since_last_opt_.size()) +
      ", total_values=" + std::to_string(values_.size()) +
      ", keyTimestampMap_size=" + std::to_string(keyTimestampMap_.size()) +
      ", keys_sent=" + std::to_string(keys_sent_to_smoother_.size()) +
      ", first_state_priors_added_=" + std::to_string(first_state_priors_added_));

  if (values_.empty()) {
    appPtr_->getLogger().warn(
        "GraphTimeCentricKimera::buildIncrementalUpdate: no values in graph, returning false");
    return false;
  }

  // Step 1: Add SmartFactors WITH marginalization check (mirrors Kimera VioBackend::optimize)
  // This is the CRITICAL fix: check if factors still exist BEFORE trying to update them
  if (new_smart_factor_lmk_ids) {
    new_smart_factor_lmk_ids->clear();
  }
  
  std::vector<LandmarkId> marginalized_landmarks;
  
  for (const auto& [lmk_id, factor_ptr] : new_smart_stereo_factors_) {
    // Find the old factor and its slot
    auto old_it = old_smart_stereo_factors_.find(lmk_id);
    if (old_it == old_smart_stereo_factors_.end()) {
      // New factor with no history - just add it
      new_factors->push_back(factor_ptr);
      if (new_smart_factor_lmk_ids) {
        new_smart_factor_lmk_ids->push_back(lmk_id);
      }
      continue;
    }
    
    const Slot slot = old_it->second.second;
    
    if (slot == -1) {
      // Factor never added to graph before - add it now
      new_factors->push_back(factor_ptr);
      if (new_smart_factor_lmk_ids) {
        new_smart_factor_lmk_ids->push_back(lmk_id);
      }
    } else {
      // Factor was previously in graph - check if it still exists
      // CRITICAL: This prevents the "invalid key" crash when states are marginalized
      if (smoother_graph && smoother_graph->exists(slot)) {
        // Factor still exists - safe to update
        new_factors->push_back(factor_ptr);
        if (new_smart_factor_lmk_ids) {
          new_smart_factor_lmk_ids->push_back(lmk_id);
        }
      } else if (!smoother_graph) {
        // No smoother graph provided - assume factor exists (backward compatibility)
        new_factors->push_back(factor_ptr);
        if (new_smart_factor_lmk_ids) {
          new_smart_factor_lmk_ids->push_back(lmk_id);
        }
      } else {
        // Factor was MARGINALIZED OUT - skip this update and clean up
        appPtr_->getLogger().info(
            "GraphTimeCentricKimera: Landmark " + std::to_string(lmk_id) +
            " references marginalized states (slot " + std::to_string(slot) +
            " no longer exists), skipping update");
        marginalized_landmarks.push_back(lmk_id);
      }
    }
  }
  
  // Clean up landmarks that reference marginalized states
  for (LandmarkId lmk_id : marginalized_landmarks) {
    old_smart_stereo_factors_.erase(lmk_id);
    stereo_feature_tracks_.erase(lmk_id);
    // Also remove from new_smart_stereo_factors_ to avoid re-adding
    new_smart_stereo_factors_.erase(lmk_id);
  }
  
  if (!marginalized_landmarks.empty()) {
    appPtr_->getLogger().info(
        "GraphTimeCentricKimera: Removed " + std::to_string(marginalized_landmarks.size()) +
        " landmark tracks due to marginalization");
  }
  
  // Step 2: Add all other factors (IMU, priors, etc.) after SmartFactors
  for (const auto& factor : new_factors_since_last_opt_) {
    // Skip SmartStereoProjectionPoseFactors since they're already added above
    if (boost::dynamic_pointer_cast<gtsam::SmartStereoProjectionPoseFactor>(factor)) {
      continue;  // Already added in Step 1
    }
    new_factors->push_back(factor);
  }

  if (new_factors->empty() && new_values_since_last_opt_.empty()) {
    appPtr_->getLogger().info(
        "GraphTimeCentricKimera::buildIncrementalUpdate: no new factors/values since last opt");
    return false;
  }

  // Step 3: Populate output values, filtering out keys already in the smoother
  // This prevents the "ValuesKeyAlreadyExists" error
  keys_to_finalize_.clear();
  for (auto it = new_values_since_last_opt_.begin(); it != new_values_since_last_opt_.end(); ++it) {
    if (keys_sent_to_smoother_.find(it->key) == keys_sent_to_smoother_.end()) {
      new_values->insert(it->key, it->value);
      keys_to_finalize_.insert(it->key);
    }
  }

  // CRITICAL FIX: Send timestamps for ALL keys that are new OR referenced by new factors
  // GTSAM's FixedLagSmoother requires timestamps for ALL keys to enable marginalization.
  // Previously, we only sent timestamps for keys_to_finalize_, which missed omega keys
  // that were updated in place (not in new_values but needed by GP prior factors).
  
  // Collect all keys referenced by new factors (including omega keys from GP priors)
  std::set<gtsam::Key> all_keys_in_new_factors;
  for (const auto& factor : *new_factors) {
    if (factor) {
      for (const auto& key : factor->keys()) {
        all_keys_in_new_factors.insert(key);
      }
    }
  }
  
  // Send timestamps for:
  // 1. Keys in new_values (keys_to_finalize_)
  // 2. Keys referenced by new factors but not in smoother yet
  // 3. Keys that need timestamp updates (omega keys for GP priors)
  for (gtsam::Key key : keys_to_finalize_) {
    auto new_ts_it = new_key_timestamps_since_last_opt_.find(key);
    if (new_ts_it != new_key_timestamps_since_last_opt_.end()) {
      (*new_timestamps)[key] = new_ts_it->second;
    } else {
      auto ts_it = keyTimestampMap_.find(key);
      if (ts_it != keyTimestampMap_.end()) {
        (*new_timestamps)[key] = ts_it->second;
      }
    }
  }
  
  // Add timestamps for keys in factors that aren't being sent as new values
  // (e.g., omega keys that were updated in place)
  for (gtsam::Key key : all_keys_in_new_factors) {
    // Only add if not already in new_timestamps and not already in smoother
    if (new_timestamps->find(key) == new_timestamps->end() &&
        keys_sent_to_smoother_.find(key) == keys_sent_to_smoother_.end()) {
      auto ts_it = keyTimestampMap_.find(key);
      if (ts_it != keyTimestampMap_.end()) {
        (*new_timestamps)[key] = ts_it->second;
        appPtr_->getLogger().debug(
            "GraphTimeCentricKimera: Adding timestamp for factor-referenced key " +
            gtsam::DefaultKeyFormatter(key) + " = " + std::to_string(ts_it->second));
      } else {
        appPtr_->getLogger().warn(
            "GraphTimeCentricKimera::buildIncrementalUpdate: "
            "Factor references key " + gtsam::DefaultKeyFormatter(key) +
            " but timestamp not found in keyTimestampMap_");
      }
    }
  }
  
  // Populate delete_slots with SmartFactor slots that need to be replaced
  // Pass smoother_graph to enable proper marginalization checking
  if (delete_slots) {
    std::vector<LandmarkId> marginalized_in_delete;
    auto slots_to_delete = getSmartFactorSlotsToDelete(smoother_graph, &marginalized_in_delete);
    *delete_slots = gtsam::FactorIndices(slots_to_delete.begin(), slots_to_delete.end());
    
    // DEFENSIVE CHECK: Ensure GP factors and omega priors are never in delete_slots
    // These factors should ONLY be marginalized (like IMU factors), never explicitly deleted
    if (!delete_slots->empty() && smoother_graph) {
      std::vector<Slot> slots_to_remove_from_deletion;
      for (Slot slot : *delete_slots) {
        if (smoother_graph->exists(slot)) {
          auto factor = smoother_graph->at(slot);
          if (factor) {
            // Check if this is a GP prior
            auto gp_wnoa = boost::dynamic_pointer_cast<fgo::factor::GPWNOAPrior>(factor);
            auto gp_wnoj = boost::dynamic_pointer_cast<fgo::factor::GPWNOJPrior>(factor);
            auto gp_singer = boost::dynamic_pointer_cast<fgo::factor::GPSingerPrior>(factor);
            
            // Check if this is an omega measurement prior (PriorFactor<Vector3> on W key)
            auto omega_prior = boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Vector3>>(factor);
            bool is_omega_prior = false;
            if (omega_prior && !omega_prior->keys().empty()) {
              gtsam::Key key = omega_prior->keys()[0];
              // Omega keys use symbol 'W'
              is_omega_prior = (gtsam::Symbol(key).chr() == 'W');
            }
            
            if (gp_wnoa || gp_wnoj || gp_singer || is_omega_prior) {
              appPtr_->getLogger().error(
                  std::string("CRITICAL: GP factor or omega prior in delete_slots! Slot=") + 
                  std::to_string(slot) + " - Removing from delete_slots (should only be marginalized)");
              slots_to_remove_from_deletion.push_back(slot);
            }
          }
        }
      }
      
      // Remove GP factors from delete_slots
      for (Slot slot : slots_to_remove_from_deletion) {
        delete_slots->erase(
            std::remove(delete_slots->begin(), delete_slots->end(), slot),
            delete_slots->end());
      }
    }
    
    // Clean up any additional landmarks detected during delete_slots computation
    for (LandmarkId lmk_id : marginalized_in_delete) {
      old_smart_stereo_factors_.erase(lmk_id);
      stereo_feature_tracks_.erase(lmk_id);
    }
    
    // CRITICAL FIX: Clean up all SmartFactor mappings where the slot no longer exists
    // This prevents stale slot tracking when GTSAM reuses slot numbers
    if (smoother_graph) {
      std::vector<LandmarkId> stale_landmarks;
      for (const auto& [lmk_id, factor_slot_pair] : old_smart_stereo_factors_) {
        const Slot slot = factor_slot_pair.second;
        const auto& factor_ptr = factor_slot_pair.first;
        
        if (slot != -1 && !smoother_graph->exists(slot)) {
          // Slot no longer exists - factor was deleted/marginalized
          stale_landmarks.push_back(lmk_id);
        } else if (slot != -1 && smoother_graph->exists(slot)) {
          // Slot exists - verify it's still our factor
          auto current_factor = smoother_graph->at(slot);
          if (!current_factor || current_factor.get() != factor_ptr.get()) {
            // Different factor at this slot - ours was replaced
            stale_landmarks.push_back(lmk_id);
          }
        }
      }
      
      for (LandmarkId lmk_id : stale_landmarks) {
        old_smart_stereo_factors_.erase(lmk_id);
        stereo_feature_tracks_.erase(lmk_id);
      }
      
      if (!stale_landmarks.empty()) {
        appPtr_->getLogger().info(
            "GraphTimeCentricKimera: Cleaned up " + std::to_string(stale_landmarks.size()) +
            " stale SmartFactor mappings");
      }
    }
    
    if (!delete_slots->empty()) {
      appPtr_->getLogger().info(
          "GraphTimeCentricKimera::buildIncrementalUpdate: " +
          std::to_string(delete_slots->size()) + " SmartFactor slots to delete");
    }
  }

  // Log detailed summary of what's being sent to the smoother
  appPtr_->getLogger().info(
      std::string("GraphTimeCentricKimera::buildIncrementalUpdate: SENDING TO SMOOTHER - ") +
      "factors=" + std::to_string(new_factors->size()) +
      ", values=" + std::to_string(new_values->size()) +
      ", timestamps=" + std::to_string(new_timestamps->size()) +
      ", delete_slots=" + std::to_string(delete_slots ? delete_slots->size() : 0));
  
  if (!new_values->empty()) {
    std::stringstream new_keys_ss;
    for (const auto& kv : *new_values) {
      new_keys_ss << gtsam::DefaultKeyFormatter(kv.key) << " ";
    }
    appPtr_->getLogger().info(
        "GraphTimeCentricKimera::buildIncrementalUpdate: New keys being sent to smoother: " + new_keys_ss.str());
  }

  return true;
}

std::vector<GraphTimeCentricKimera::Slot> GraphTimeCentricKimera::getSmartFactorSlotsToDelete(
    const gtsam::NonlinearFactorGraph* smoother_graph,
    std::vector<LandmarkId>* landmarks_to_remove) {
  std::vector<Slot> slots_to_delete;
  slots_to_delete.reserve(new_smart_stereo_factors_.size() + marginalized_keys_.size());

  // PART 1: Delete slots for updated smart factors (existing logic)
  // Mirrors Kimera-VIO VioBackend::optimize():
  // For each updated smart factor, delete the slot of the old factor ONLY if it still
  // exists in the smoother graph. If it no longer exists, it was marginalized out
  // and we should drop it from tracking without deleting anything.
  for (const auto& [lmk_id, /*new_factor*/ _] : new_smart_stereo_factors_) {
    (void)_;  // unused
    auto old_it = old_smart_stereo_factors_.find(lmk_id);
    if (old_it == old_smart_stereo_factors_.end()) {
      continue;
    }

    const Slot slot = old_it->second.second;
    if (slot == -1) {
      continue;
    }

    // CRITICAL CHECK: Only delete if factor still exists in smoother
    if (smoother_graph && smoother_graph->exists(slot)) {
      slots_to_delete.push_back(slot);
    } else if (!smoother_graph) {
      // No smoother graph provided - assume exists (backward compatibility)
      slots_to_delete.push_back(slot);
    } else {
      // Factor was marginalized - track for cleanup
      if (landmarks_to_remove) {
        landmarks_to_remove->push_back(lmk_id);
      }
    }
  }

  // PART 2: Delete slots for SmartFactors that reference marginalized keys
  // This is the missing piece that causes linear factor growth
  if (smoother_graph && !marginalized_keys_.empty()) {
    for (const auto& [lmk_id, factor_slot_pair] : old_smart_stereo_factors_) {
      const auto& factor = factor_slot_pair.first;
      const Slot slot = factor_slot_pair.second;

      if (slot == -1 || !smoother_graph->exists(slot)) {
        continue;  // Already deleted or not in graph
      }

      // CRITICAL: Verify the factor at this slot is still the SmartFactor we expect
      // Slot numbers can be reused - a GP/omega prior might now occupy this slot
      auto factor_at_slot = smoother_graph->at(slot);
      if (!factor_at_slot || factor_at_slot.get() != factor.get()) {
        // Different factor at this slot now (slot was reused) - mark for cleanup
        if (landmarks_to_remove) {
          landmarks_to_remove->push_back(lmk_id);
        }
        continue;
      }

      // Check if this SmartFactor references any marginalized keys
      bool references_marginalized = false;
      for (const gtsam::Key& key : factor->keys()) {
        if (marginalized_keys_.find(key) != marginalized_keys_.end()) {
          references_marginalized = true;
          break;
        }
      }

      if (references_marginalized) {
        slots_to_delete.push_back(slot);
        if (landmarks_to_remove) {
          landmarks_to_remove->push_back(lmk_id);
        }
        appPtr_->getLogger().info(
            "GraphTimeCentricKimera: Deleting SmartFactor for landmark " +
            std::to_string(lmk_id) + " (slot " + std::to_string(slot) +
            ") because it references marginalized keys");
      }
    }
  }

  return slots_to_delete;
}

void GraphTimeCentricKimera::finalizeIncrementalUpdate() {
  appPtr_->getLogger().info(
      std::string("GraphTimeCentricKimera::finalizeIncrementalUpdate: ENTERING - ") +
      "keys_to_finalize_=" + std::to_string(keys_to_finalize_.size()) +
      ", keys_sent_to_smoother_=" + std::to_string(keys_sent_to_smoother_.size()) +
      ", new_values_since_last_opt_=" + std::to_string(new_values_since_last_opt_.size()));
  
  // 1. Clear factors that were just sent
  new_factors_since_last_opt_.resize(0);
  
  // Clear new_smart_stereo_factors_ as they've been added to the graph
  // The old_smart_stereo_factors_ still tracks them with their slot numbers
  new_smart_stereo_factors_.clear();

  // 2. Mark values as sent and remove them from incremental buffer
  std::stringstream finalized_keys_ss;
  for (gtsam::Key key : keys_to_finalize_) {
    keys_sent_to_smoother_.insert(key);
    finalized_keys_ss << gtsam::DefaultKeyFormatter(key) << " ";
    
    // Remove from staging buffers if they exist there
    if (new_values_since_last_opt_.exists(key)) {
      new_values_since_last_opt_.erase(key);
    }
    new_key_timestamps_since_last_opt_.erase(key);
  }
  
  appPtr_->getLogger().info(
      "GraphTimeCentricKimera::finalizeIncrementalUpdate: Finalized keys: " + finalized_keys_ss.str());
  
  keys_to_finalize_.clear();
  
  appPtr_->getLogger().info(
      "GraphTimeCentricKimera: finalized incremental update, " +
      std::to_string(keys_sent_to_smoother_.size()) + " keys tracked (before marginalization check).");
  
  // Clear marginalized keys after they've been processed for factor deletion
  marginalized_keys_.clear();
}

void GraphTimeCentricKimera::updateMarginalizedKeys(const std::set<gtsam::Key>& current_smoother_keys) {
  // Find keys that were sent but are no longer in the smoother (they were marginalized)
  std::vector<gtsam::Key> newly_marginalized_keys;
  
  for (const auto& key : keys_sent_to_smoother_) {
    if (current_smoother_keys.find(key) == current_smoother_keys.end()) {
      newly_marginalized_keys.push_back(key);
      marginalized_keys_.insert(key);
    }
  }
  
  // Remove marginalized keys from active tracking
  for (const auto& key : newly_marginalized_keys) {
    keys_sent_to_smoother_.erase(key);
    
    // CRITICAL FIX: Clean up keyTimestampMap_ and values_ to prevent unbounded growth.
    // When keys are marginalized, they should be removed from our tracking structures
    // to ensure consistent state and prevent memory growth.
    keyTimestampMap_.erase(key);
    
    // Also clean up values_ to prevent unbounded growth
    if (values_.exists(key)) {
      values_.erase(key);
    }
  }
  
  if (!newly_marginalized_keys.empty()) {
    std::stringstream marg_ss;
    for (const auto& key : newly_marginalized_keys) {
      marg_ss << gtsam::DefaultKeyFormatter(key) << " ";
    }
    appPtr_->getLogger().info(
        "GraphTimeCentricKimera: Removed " + std::to_string(newly_marginalized_keys.size()) +
        " marginalized keys: " + marg_ss.str() +
        ". Active keys in smoother: " + std::to_string(keys_sent_to_smoother_.size()) +
        ", keyTimestampMap_size: " + std::to_string(keyTimestampMap_.size()) +
        ", values_size: " + std::to_string(values_.size()));
  }
}

// ========================================================================
// SMART FACTOR MANAGEMENT
// ========================================================================

namespace {
// Remove any factors containing a specific key from a factor graph.
static void deleteAllFactorsWithKey(const gtsam::Key& key,
                                    const gtsam::NonlinearFactorGraph& in,
                                    gtsam::NonlinearFactorGraph* out) {
  if (!out) {
    return;
  }
  out->resize(0);
  out->reserve(in.size());
  for (const auto& f : in) {
    if (!f) {
      continue;
    }
    const auto& keys = f->keys();
    if (std::find(keys.begin(), keys.end(), key) == keys.end()) {
      out->push_back(f);
    }
  }
}

static bool deleteKeyFromValues(const gtsam::Key& key,
                                const gtsam::Values& in,
                                gtsam::Values* out) {
  if (!out) {
    return false;
  }
  out->clear();
  bool deleted = false;
  for (const auto& kv : in) {
    if (kv.key == key) {
      deleted = true;
      continue;
    }
    out->insert(kv.key, kv.value);
  }
  return deleted;
}

static bool deleteKeyFromTimestamps(const gtsam::Key& key,
                                    const fgo::solvers::FixedLagSmoother::KeyTimestampMap& in,
                                    fgo::solvers::FixedLagSmoother::KeyTimestampMap* out) {
  if (!out) {
    return false;
  }
  out->clear();
  bool deleted = false;
  for (const auto& [k, ts] : in) {
    if (k == key) {
      deleted = true;
      continue;
    }
    (*out)[k] = ts;
  }
  return deleted;
}

static void findSlotsOfFactorsWithKey(const gtsam::Key& key,
                                      const gtsam::NonlinearFactorGraph& graph,
                                      std::vector<size_t>* slots) {
  if (!slots) {
    return;
  }
  slots->clear();
  for (size_t i = 0; i < graph.size(); ++i) {
    if (!graph[i]) {
      continue;
    }
    const auto& keys = graph[i]->keys();
    if (std::find(keys.begin(), keys.end(), key) != keys.end()) {
      slots->push_back(i);
    }
  }
}
}  // namespace

size_t GraphTimeCentricKimera::cleanCheiralityLandmarks(
    const gtsam::Symbol& lmk_symbol,
    const gtsam::NonlinearFactorGraph& graph_in_smoother,
    const gtsam::NonlinearFactorGraph& new_factors,
    const gtsam::Values& new_values,
    const fgo::solvers::FixedLagSmoother::KeyTimestampMap& new_timestamps,
    const gtsam::FactorIndices& delete_slots,
    gtsam::NonlinearFactorGraph* new_factors_out,
    gtsam::Values* new_values_out,
    fgo::solvers::FixedLagSmoother::KeyTimestampMap* new_timestamps_out,
    gtsam::FactorIndices* delete_slots_out) {
  if (!new_factors_out || !new_values_out || !new_timestamps_out || !delete_slots_out) {
    appPtr_->getLogger().error(
        "GraphTimeCentricKimera::cleanCheiralityLandmarks: null output args");
    return 0;
  }

  // Kimera-VIO uses point landmarks with symbol 'l'. For smart factors we track by LandmarkId.
  // In this integration, the lmk id is carried in the symbol index.
  const LandmarkId lmk_id = static_cast<LandmarkId>(lmk_symbol.index());
  const gtsam::Key lmk_key = lmk_symbol.key();

  // 1) Remove any pending new factors that reference this landmark key (if any).
  // (In our current GTC integration, we do not insert landmark values, but we still
  //  apply the same “remove from new_factors/new_values/new_timestamps” pattern.)
  deleteAllFactorsWithKey(lmk_key, new_factors, new_factors_out);

  // 2) Remove key from new values/timestamps if present.
  gtsam::Values tmp_values;
  bool deleted_values = deleteKeyFromValues(lmk_key, new_values, &tmp_values);
  *new_values_out = tmp_values;

  fgo::solvers::FixedLagSmoother::KeyTimestampMap tmp_ts;
  bool deleted_ts = deleteKeyFromTimestamps(lmk_key, new_timestamps, &tmp_ts);
  *new_timestamps_out = tmp_ts;

  // Match VioBackend invariant: if removed from values, removed from timestamps too.
  if (deleted_values != deleted_ts) {
    appPtr_->getLogger().warn(
        "GraphTimeCentricKimera::cleanCheiralityLandmarks: inconsistent deletion for key " +
        std::string(1, lmk_symbol.chr()) + std::to_string(lmk_symbol.index()));
  }

  // 3) Delete from current (smoother) graph all factors that touch this key.
  *delete_slots_out = delete_slots;
  
  // Optimization: Instead of searching the whole graph, use our tracked slot.
  auto factor_it = old_smart_stereo_factors_.find(lmk_id);
  if (factor_it != old_smart_stereo_factors_.end()) {
    Slot slot = factor_it->second.second;
    if (slot != -1) {
      // Check if this slot is already in delete_slots_out to avoid duplicates
      if (std::find(delete_slots_out->begin(), delete_slots_out->end(), slot) == delete_slots_out->end()) {
        delete_slots_out->push_back(slot);
        appPtr_->getLogger().info(
            "GraphTimeCentricKimera::cleanCheiralityLandmarks: adding tracked slot " + 
            std::to_string(slot) + " for removal");
      }
    }
  }

  // 4) Bookkeeping: remove from our smart-factor tracking.
  removeLandmark(lmk_id);

  appPtr_->getLogger().warn(
      "GraphTimeCentricKimera::cleanCheiralityLandmarks: removed landmark " +
      std::to_string(lmk_id) + " (symbol=" + std::string(1, lmk_symbol.chr()) +
      std::to_string(lmk_symbol.index()) + ")");

  return 1;
}

bool GraphTimeCentricKimera::removeLandmark(const LandmarkId& lm_id) {
  std::lock_guard<std::mutex> lock(landmark_mutex_);
  bool removed_any = false;

  // Remove feature track.
  auto ft_it = stereo_feature_tracks_.find(lm_id);
  if (ft_it != stereo_feature_tracks_.end()) {
    stereo_feature_tracks_.erase(ft_it);
    removed_any = true;
  }

  // Remove factor tracking.
  new_smart_stereo_factors_.erase(lm_id);
  auto old_it = old_smart_stereo_factors_.find(lm_id);
  if (old_it != old_smart_stereo_factors_.end()) {
    old_smart_stereo_factors_.erase(old_it);
    removed_any = true;
  }

  // Remove legacy maps if used.
  new_smart_factors_.erase(lm_id);
  old_smart_factors_.erase(lm_id);

  // Remove id mapping if present.
  kimera_to_plugin_id_.erase(lm_id);

  return removed_any;
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
    }
    // ALWAYS stage for the next optimization to ensure smoother has the latest values
    if (new_values_since_last_opt_.exists(pose_key)) {
      new_values_since_last_opt_.update(pose_key, predicted_state.state.pose());
    } else {
      new_values_since_last_opt_.insert(pose_key, predicted_state.state.pose());
    }
    new_key_timestamps_since_last_opt_[pose_key] = timestamp;
    
    if (values_.exists(vel_key)) {
      values_.update(vel_key, predicted_state.state.velocity());
    } else {
      values_.insert(vel_key, predicted_state.state.velocity());
    }
    if (new_values_since_last_opt_.exists(vel_key)) {
      new_values_since_last_opt_.update(vel_key, predicted_state.state.velocity());
    } else {
      new_values_since_last_opt_.insert(vel_key, predicted_state.state.velocity());
    }
    new_key_timestamps_since_last_opt_[vel_key] = timestamp;
    
    if (values_.exists(bias_key)) {
      values_.update(bias_key, predicted_state.imuBias);
    } else {
      values_.insert(bias_key, predicted_state.imuBias);
    }
    if (new_values_since_last_opt_.exists(bias_key)) {
      new_values_since_last_opt_.update(bias_key, predicted_state.imuBias);
    } else {
      new_values_since_last_opt_.insert(bias_key, predicted_state.imuBias);
    }
    new_key_timestamps_since_last_opt_[bias_key] = timestamp;
    
    // Update keyTimestampMap
    keyTimestampMap_[pose_key] = timestamp;
    keyTimestampMap_[vel_key] = timestamp;
    keyTimestampMap_[bias_key] = timestamp;
    
    // Update currentKeyIndexTimestampMap_
    currentKeyIndexTimestampMap_.insert(std::make_pair(state_idx, timestamp));
    
        
        // If GP factors enabled, also insert omega keys
        if (kimeraParams_.addGPMotionPriors) {
          gtsam::Key omega_key = W(state_idx);
          values_.insert(omega_key, predicted_state.omega);
          new_values_since_last_opt_.insert(omega_key, predicted_state.omega);
          keyTimestampMap_[omega_key] = timestamp;
          new_key_timestamps_since_last_opt_[omega_key] = timestamp;
          
          // Only create acceleration keys for Full GP variants
          // where acceleration is an optimization variable (WNOJFull, SingerFull)
          // not for standard variants (WNOA, WNOJ, Singer) where acceleration
          // is computed internally
          if (kimeraParams_.gpType == fgo::data::GPModelType::WNOJFull ||
              kimeraParams_.gpType == fgo::data::GPModelType::SingerFull) {
            gtsam::Key acc_key = A(state_idx);
            // Get acceleration from buffer if available, otherwise use zero
            gtsam::Vector6 acc = gtsam::Vector6::Zero();
            if (state_idx > 0 && accBuffer_.size() > (state_idx - 1)) {
              acc = accBuffer_.get_buffer_from_id(state_idx - 1);
            }
            values_.insert(acc_key, acc);
            new_values_since_last_opt_.insert(acc_key, acc);
            keyTimestampMap_[acc_key] = timestamp;
            new_key_timestamps_since_last_opt_[acc_key] = timestamp;
          }
        }    
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
    
    // Update or insert into values_
    if (values_.exists(pose_key)) {
      values_.update(pose_key, pose);
    } else {
      values_.insert(pose_key, pose);
    }

    // ALWAYS stage for the next optimization to ensure smoother has the latest values.
    // FixedLagSmoother is fine with updates to existing keys.
    if (new_values_since_last_opt_.exists(pose_key)) {
      new_values_since_last_opt_.update(pose_key, pose);
    } else {
      new_values_since_last_opt_.insert(pose_key, pose);
    }
    
    // Always ensure timestamp is staged
    if (keyTimestampMap_.find(pose_key) != keyTimestampMap_.end()) {
      new_key_timestamps_since_last_opt_[pose_key] = keyTimestampMap_[pose_key];
    }

    if (values_.exists(vel_key)) {
      values_.update(vel_key, velocity);
    } else {
      values_.insert(vel_key, velocity);
    }
    if (new_values_since_last_opt_.exists(vel_key)) {
      new_values_since_last_opt_.update(vel_key, velocity);
    } else {
      new_values_since_last_opt_.insert(vel_key, velocity);
    }
    if (keyTimestampMap_.find(vel_key) != keyTimestampMap_.end()) {
      new_key_timestamps_since_last_opt_[vel_key] = keyTimestampMap_[vel_key];
    }

    if (values_.exists(bias_key)) {
      values_.update(bias_key, bias);
    } else {
      values_.insert(bias_key, bias);
    }
    if (new_values_since_last_opt_.exists(bias_key)) {
      new_values_since_last_opt_.update(bias_key, bias);
    } else {
      new_values_since_last_opt_.insert(bias_key, bias);
    }
    if (keyTimestampMap_.find(bias_key) != keyTimestampMap_.end()) {
      new_key_timestamps_since_last_opt_[bias_key] = keyTimestampMap_[bias_key];
    }
    
    // If GP motion priors are enabled, we need to ensure omega keys are updated.
    // Acceleration keys are ONLY needed for Full GP variants.
    if (kimeraParams_.addGPMotionPriors) {
      gtsam::Key omega_key = W(state_idx);

      // We might have omega from setOmegaForState (state_omega_ map)
      gtsam::Vector3 omega = gtsam::Vector3::Zero();
      if (state_omega_.find(state_idx) != state_omega_.end()) {
        omega = state_omega_[state_idx].omega;
      }

      if (values_.exists(omega_key)) {
        values_.update(omega_key, omega);
      } else {
        values_.insert(omega_key, omega);
      }
      if (new_values_since_last_opt_.exists(omega_key)) {
        new_values_since_last_opt_.update(omega_key, omega);
      } else {
        new_values_since_last_opt_.insert(omega_key, omega);
      }
      if (keyTimestampMap_.find(omega_key) != keyTimestampMap_.end()) {
        new_key_timestamps_since_last_opt_[omega_key] = keyTimestampMap_[omega_key];
      }

      // Only handle acceleration keys for Full GP variants where acceleration
      // is an optimization variable (WNOJFull, SingerFull)
      if (kimeraParams_.gpType == fgo::data::GPModelType::WNOJFull ||
          kimeraParams_.gpType == fgo::data::GPModelType::SingerFull) {
        gtsam::Key acc_key = A(state_idx);
        
        // Acceleration (for Full GP variants)
        gtsam::Vector6 acc = gtsam::Vector6::Zero();
        if (state_idx > 0 && accBuffer_.size() > (state_idx - 1)) {
           acc = accBuffer_.get_buffer_from_id(state_idx - 1);
        }

        if (values_.exists(acc_key)) {
          values_.update(acc_key, acc);
        } else {
          values_.insert(acc_key, acc);
        }
        if (new_values_since_last_opt_.exists(acc_key)) {
          new_values_since_last_opt_.update(acc_key, acc);
        } else {
          new_values_since_last_opt_.insert(acc_key, acc);
        }
        if (keyTimestampMap_.find(acc_key) != keyTimestampMap_.end()) {
          new_key_timestamps_since_last_opt_[acc_key] = keyTimestampMap_[acc_key];
        }
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
    // this->push_back(prior_pose); // Disabled for Stateless GTC
    new_factors_since_last_opt_.push_back(prior_pose);
    
    auto prior_vel = gtsam::PriorFactor<gtsam::Vector3>(vel_key, velocity, noise_init_vel_prior);
    // this->push_back(prior_vel); // Disabled for Stateless GTC
    new_factors_since_last_opt_.push_back(prior_vel);
    
    auto prior_bias = gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(bias_key, bias, imu_bias_prior_noise);
    // this->push_back(prior_bias); // Disabled for Stateless GTC
    new_factors_since_last_opt_.push_back(prior_bias);

    // If GP priors are enabled, we also need to add priors for omega and acceleration
    // to ensure the first state is fully constrained.
    if (kimeraParams_.addGPMotionPriors) {
      gtsam::Key omega_key = W(state_idx);
      gtsam::Key acc_key = A(state_idx);

      if (values_.exists(omega_key)) {
        gtsam::Vector3 omega = values_.at<gtsam::Vector3>(omega_key);
        // Use a weak prior for omega (e.g., 0.1 rad/s)
        auto noise_omega = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
        new_factors_since_last_opt_.push_back(
            gtsam::PriorFactor<gtsam::Vector3>(omega_key, omega, noise_omega));
      }

      if (values_.exists(acc_key)) {
        gtsam::Vector6 acc = values_.at<gtsam::Vector6>(acc_key);
        // Use a weak prior for acceleration (e.g., 0.1 m/s^2)
        auto noise_acc = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
        new_factors_since_last_opt_.push_back(
            gtsam::PriorFactor<gtsam::Vector6>(acc_key, acc, noise_acc));
      }
    }

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

// ============================================================================
// STEREO VISUAL FACTOR IMPLEMENTATION
// ============================================================================

void GraphTimeCentricKimera::setStereoCalibration(
    const gtsam::Cal3_S2Stereo::shared_ptr& stereo_cal,
    const gtsam::Pose3& B_Pose_leftCam,
    const gtsam::SharedNoiseModel& smart_noise,
    const gtsam::SmartProjectionParams& smart_params) {
  stereo_cal_ = stereo_cal;
  B_Pose_leftCam_ = B_Pose_leftCam;
  
  // Use pre-initialized smart factor params from VioBackend
  // This ensures the same parameters as the standard VioBackend path
  // (initialized from BackendParams.yaml via setFactorsParams)
  smart_noise_ = smart_noise;
  smart_stereo_params_ = smart_params;
  
  appPtr_->getLogger().info("GraphTimeCentricKimera: Stereo calibration set - baseline: " + 
                            std::to_string(stereo_cal->baseline()) +
                            " (using pre-initialized smart factor params from VioBackend)");
}

void GraphTimeCentricKimera::setGPPriorParams(
    const gtsam::SharedNoiseModel& gp_qc_model,
    const gtsam::Matrix6& gp_ad_matrix,
    const gtsam::SharedNoiseModel& gp_acc_prior_noise) {
  gp_qc_model_ = gp_qc_model;
  gp_ad_matrix_ = gp_ad_matrix;
  gp_acc_prior_noise_ = gp_acc_prior_noise;
  gp_params_initialized_ = true;
  
  appPtr_->getLogger().info("GraphTimeCentricKimera: GP motion prior params set - "
                            "Qc model, ad matrix (diag=" + 
                            std::to_string(gp_ad_matrix_(0,0)) + "/" + 
                            std::to_string(gp_ad_matrix_(3,3)) + 
                            "), acc prior (pre-initialized from VioBackend)");
}

bool GraphTimeCentricKimera::setOmegaForState(size_t state_id, const OmegaAtState& omega_state) {
  try {
    if (!omega_state.valid) {
      appPtr_->getLogger().warn("GraphTimeCentricKimera: Invalid OmegaAtState for state " + 
                                std::to_string(state_id));
      return false;
    }
    
    state_omega_[state_id] = omega_state;
    appPtr_->getLogger().debug("GraphTimeCentricKimera: Set omega for state " + 
                               std::to_string(state_id) + 
                               ": [" + std::to_string(omega_state.omega.x()) + ", " +
                               std::to_string(omega_state.omega.y()) + ", " +
                               std::to_string(omega_state.omega.z()) + "] rad/s" +
                               " (raw_gyro=[" + std::to_string(omega_state.gyro_raw.x()) + "," +
                               std::to_string(omega_state.gyro_raw.y()) + "," +
                               std::to_string(omega_state.gyro_raw.z()) + "])");
    return true;
  } catch (const std::exception& e) {
    appPtr_->getLogger().error("GraphTimeCentricKimera: Exception setting omega for state " + 
                               std::to_string(state_id) + ": " + std::string(e.what()));
    return false;
  }
}

size_t GraphTimeCentricKimera::addStereoMeasurementsToGraph(
    FrameId frame_id,
    const std::vector<StereoMeasurement>& stereo_measurements) {
  
  if (!stereo_cal_) {
    appPtr_->getLogger().error("GraphTimeCentricKimera: Stereo calibration not set, cannot add visual factors");
    return 0;
  }
  
  size_t n_new_landmarks = 0;
  size_t n_updated_landmarks = 0;
  
  for (const auto& meas : stereo_measurements) {
    const LandmarkId lmk_id = meas.first;
    const gtsam::StereoPoint2& stereo_px = meas.second;
    
    // Check if we have an existing track for this landmark
    auto track_it = stereo_feature_tracks_.find(lmk_id);
    
    if (track_it == stereo_feature_tracks_.end()) {
      // New landmark - create feature track
      StereoFeatureTrack new_track;
      new_track.observations.emplace_back(frame_id, stereo_px);
      new_track.in_ba_graph = false;
      stereo_feature_tracks_[lmk_id] = new_track;
    } else {
      // Existing landmark - add observation
      track_it->second.observations.emplace_back(frame_id, stereo_px);
    }
  }
  
  // Now add/update landmarks in graph (mirrors VioBackend::addLandmarksToGraph)
  for (const auto& meas : stereo_measurements) {
    const LandmarkId lmk_id = meas.first;
    auto track_it = stereo_feature_tracks_.find(lmk_id);
    
    if (track_it == stereo_feature_tracks_.end()) continue;
    
    StereoFeatureTrack& ft = track_it->second;
    
    // Only add tracks with >= minObservations (default 2)
    if (ft.observations.size() < kimeraParams_.minObservations) {
      continue;
    }
    
    if (!ft.in_ba_graph) {
      // New landmark - add to graph
      ft.in_ba_graph = true;
      if (addStereoLandmarkToGraph(lmk_id, ft)) {
        ++n_new_landmarks;
      }
    } else {
      // Update existing landmark with new observation
      const auto& last_obs = ft.observations.back();
      if (updateStereoLandmarkInGraph(lmk_id, last_obs.first, last_obs.second)) {
        ++n_updated_landmarks;
      }
    }
  }
  
  appPtr_->getLogger().debug("GraphTimeCentricKimera: Added " + std::to_string(n_new_landmarks) +
                             " new landmarks, updated " + std::to_string(n_updated_landmarks) + 
                             " landmarks at frame " + std::to_string(frame_id));
  
  return n_new_landmarks + n_updated_landmarks;
}

bool GraphTimeCentricKimera::addStereoLandmarkToGraph(
    const LandmarkId& lmk_id,
    const StereoFeatureTrack& track) {
  
  try {
    // Create new SmartStereoFactor (mirrors VioBackend::addLandmarkToGraph)
    SmartStereoFactorPtr new_factor = boost::make_shared<gtsam::SmartStereoProjectionPoseFactor>(
        smart_noise_, smart_stereo_params_, B_Pose_leftCam_);
    
    // Add all observations to the factor
    for (const auto& obs : track.observations) {
      const FrameId& frame_id = obs.first;
      const gtsam::StereoPoint2& measurement = obs.second;
      const gtsam::Symbol pose_symbol('x', frame_id);
      new_factor->add(measurement, pose_symbol, stereo_cal_);
    }
    
    // Store in new smart factors (to be added to graph during optimization)
    // Note: SmartFactors are handled separately from new_factors_since_last_opt_ 
    // to allow for explicit slot deletion/replacement in the smoother.
    new_smart_stereo_factors_[lmk_id] = new_factor;
    
    // Also store in old_smart_stereo_factors_ with slot = -1 (not yet in graph)
    old_smart_stereo_factors_[lmk_id] = std::make_pair(new_factor, -1);
    
    return true;
    
  } catch (const std::exception& e) {
    appPtr_->getLogger().error("GraphTimeCentricKimera: Exception adding stereo landmark " + 
                               std::to_string(lmk_id) + ": " + std::string(e.what()));
    return false;
  }
}

bool GraphTimeCentricKimera::updateStereoLandmarkInGraph(
    const LandmarkId& lmk_id,
    FrameId frame_id,
    const gtsam::StereoPoint2& measurement) {
  
  try {
    // Find existing factor
    auto old_it = old_smart_stereo_factors_.find(lmk_id);
    if (old_it == old_smart_stereo_factors_.end()) {
      appPtr_->getLogger().error("GraphTimeCentricKimera: Landmark " + std::to_string(lmk_id) + 
                                 " not found in old_smart_stereo_factors_");
      return false;
    }
    
    const SmartStereoFactorPtr& old_factor = old_it->second.first;
    Slot slot = old_it->second.second;
    
    // If slot == -1, the factor hasn't been integrated into the graph yet.
    // In our Stateless GTC architecture, this happens when multiple updates 
    // arrive within the same optimization cycle before the smoother has 
    // assigned a slot. We should just update the pending factor.
    if (slot == -1) {
      appPtr_->getLogger().debug("GraphTimeCentricKimera: Landmark " + std::to_string(lmk_id) + 
                               " is pending addition (slot -1), updating pre-optimization.");
      
      SmartStereoFactorPtr new_factor = boost::make_shared<gtsam::SmartStereoProjectionPoseFactor>(*old_factor);
      const gtsam::Symbol pose_symbol('x', frame_id);
      new_factor->add(measurement, pose_symbol, stereo_cal_);
      
      new_smart_stereo_factors_[lmk_id] = new_factor;
      old_it->second.first = new_factor;
      return true;
    }
    
    // Clone old factor and add new observation (mirrors VioBackend::updateLandmarkInGraph)
    SmartStereoFactorPtr new_factor = boost::make_shared<gtsam::SmartStereoProjectionPoseFactor>(*old_factor);
    
    const gtsam::Symbol pose_symbol('x', frame_id);
    new_factor->add(measurement, pose_symbol, stereo_cal_);
    
    // Store updated factor in new_smart_stereo_factors_ (to be added in this optimization cycle)
    // This triggers slot deletion of old factor and addition of new factor
    new_smart_stereo_factors_[lmk_id] = new_factor;
    
    // Update old_smart_stereo_factors_ to point to new factor (slot will be updated after optimization)
    old_it->second.first = new_factor;
    
    return true;
    
  } catch (const std::exception& e) {
    appPtr_->getLogger().error("GraphTimeCentricKimera: Exception updating stereo landmark " + 
                               std::to_string(lmk_id) + ": " + std::string(e.what()));
    return false;
  }
}

void GraphTimeCentricKimera::updateSmartFactorSlots(
    const std::vector<LandmarkId>& lmk_ids,
    const std::vector<Slot>& factor_slots) {
  
  if (lmk_ids.size() != factor_slots.size()) {
    appPtr_->getLogger().error("GraphTimeCentricKimera: Mismatch between lmk_ids and factor_slots size");
    return;
  }
  
  // Update slot tracking for SmartFactors (mirrors VioBackend::updateNewSmartFactorsSlots)
  for (size_t i = 0; i < lmk_ids.size(); ++i) {
    const LandmarkId& lmk_id = lmk_ids[i];
    const Slot& slot = factor_slots[i];
    
    auto it = old_smart_stereo_factors_.find(lmk_id);
    if (it != old_smart_stereo_factors_.end()) {
      // Update the slot number so it can be deleted on next update
      it->second.second = slot;
      
      
    } else {
      appPtr_->getLogger().warn(
          "GraphTimeCentricKimera: Landmark " + std::to_string(lmk_id) + 
          " not found in old_smart_stereo_factors_ during slot update");
    }
  }
  
  appPtr_->getLogger().info(
      "GraphTimeCentricKimera: Updated slots for " + 
      std::to_string(lmk_ids.size()) + " SmartFactors");
}

void GraphTimeCentricKimera::removeLandmarkTrack(LandmarkId lmk_id) {
  // Remove from feature tracks
  auto track_it = stereo_feature_tracks_.find(lmk_id);
  if (track_it != stereo_feature_tracks_.end()) {
    appPtr_->getLogger().info("GraphTimeCentricKimera: Removing landmark track " + 
                              std::to_string(lmk_id) + " (references marginalized state)");
    stereo_feature_tracks_.erase(track_it);
  }
  
  // Remove from old smart factors
  auto factor_it = old_smart_stereo_factors_.find(lmk_id);
  if (factor_it != old_smart_stereo_factors_.end()) {
    old_smart_stereo_factors_.erase(factor_it);
  }
  
  // Also remove from new smart factors if it's pending
  auto new_factor_it = new_smart_stereo_factors_.find(lmk_id);
  if (new_factor_it != new_smart_stereo_factors_.end()) {
    new_smart_stereo_factors_.erase(new_factor_it);
  }
}


bool GraphTimeCentricKimera::getCurrentGraphAndValues(
    gtsam::NonlinearFactorGraph& graph,
    gtsam::Values& values) const {
  
  // Copy the full factor graph (GraphTimeCentricKimera inherits from GraphBase which has the graph)
  graph.resize(0);
  for (size_t i = 0; i < this->size(); ++i) {
    if ((*this)[i]) {
      graph.push_back((*this)[i]);
    }
  }
  
  // Copy the full values
  values = values_;
  
  return true;
}

} // namespace fgo::graph

