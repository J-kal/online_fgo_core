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
//  KimeraIntegrationInterface implementation
//

#include "online_fgo_core/integration/KimeraIntegrationInterface.h"
#include "online_fgo_core/data/DataTypesFGO.h"
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>

namespace fgo::integration {

// Helper function to save factor graph debug information
static void saveFactorGraphDebugInfo(
    const gtsam::NonlinearFactorGraph& graph,
    const gtsam::Values& values,
    const std::string& error_type,
    fgo::core::LoggerInterface& logger) {
  
  try {
    // Create debug_run_logs directory if it doesn't exist
    const std::string debug_dir = "debug_run_logs";
    mkdir(debug_dir.c_str(), 0755);
    
    // Generate timestamp for filenames
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now = *std::localtime(&time_t_now);
    
    std::ostringstream timestamp_ss;
    timestamp_ss << std::put_time(&tm_now, "%Y%m%d_%H%M%S");
    std::string timestamp = timestamp_ss.str();
    
    std::string prefix = debug_dir + "/" + error_type + "_" + timestamp;
    
    // 1. Print factor keys to logger (easier to read than full graph)
    logger.error("=== Factor Graph Debug Info (" + error_type + ") ===");
    logger.error("Total factors: " + std::to_string(graph.size()));
    logger.error("Total values: " + std::to_string(values.size()));
    
    logger.error("\n--- Factor Keys ---");
    for (size_t i = 0; i < graph.size(); ++i) {
      if (graph[i]) {
        std::ostringstream keys_ss;
        keys_ss << "Factor " << i << ": ";
        for (const auto& key : graph[i]->keys()) {
          keys_ss << gtsam::DefaultKeyFormatter(key) << " ";
        }
        logger.error(keys_ss.str());
      }
    }
    
    logger.error("\n--- Value Keys ---");
    std::ostringstream values_ss;
    for (const auto& key_value : values) {
      values_ss << gtsam::DefaultKeyFormatter(key_value.key) << " ";
    }
    logger.error(values_ss.str());
    
    // 2. Save .g2o file (standard factor graph format)
    std::string g2o_filename = prefix + ".g2o";
    logger.error("Saving .g2o file to: " + g2o_filename);
    graph.saveGraph(g2o_filename, values);
    
    // 3. Save .dot file (GraphViz visualization format)
    std::string dot_filename = prefix + ".dot";
    logger.error("Saving .dot file to: " + dot_filename);
    std::ofstream dot_file(dot_filename);
    if (dot_file.is_open()) {
      // Use the print method with a custom formatter
      dot_file << "digraph G {" << std::endl;
      dot_file << "  // Factor Graph Visualization" << std::endl;
      dot_file << "  // Total factors: " << graph.size() << std::endl;
      dot_file << "  // Total values: " << values.size() << std::endl;
      
      // Add nodes for values
      for (const auto& key_value : values) {
        dot_file << "  " << gtsam::DefaultKeyFormatter(key_value.key) 
                << " [shape=box];" << std::endl;
      }
      
      // Add edges for factors
      for (size_t i = 0; i < graph.size(); ++i) {
        if (graph[i] && graph[i]->size() > 1) {
          auto keys = graph[i]->keys();
          for (size_t j = 1; j < keys.size(); ++j) {
            dot_file << "  " << gtsam::DefaultKeyFormatter(keys[0]) 
                    << " -> " << gtsam::DefaultKeyFormatter(keys[j])
                    << " [label=\"F" << i << "\"];" << std::endl;
          }
        }
      }
      
      dot_file << "}" << std::endl;
      dot_file.close();
    }
    
    logger.error("=== Factor Graph Debug Info saved ===\n");
    
  } catch (const std::exception& e) {
    logger.error("Failed to save factor graph debug info: " + std::string(e.what()));
  }
}

KimeraIntegrationInterface::KimeraIntegrationInterface(fgo::core::ApplicationInterface& app)
    : app_(&app) {
  app_->getLogger().info("KimeraIntegrationInterface: created");
}

bool KimeraIntegrationInterface::initialize(const KimeraIntegrationParams& params) {
  app_->getLogger().info("KimeraIntegrationInterface: initializing...");
  std::cout << "[online_fgo_core] KimeraIntegrationInterface: INITIALIZING" << std::endl;
  
  params_ = params;
  
  try {
    // Create GraphTimeCentricKimera instance
    graph_ = std::make_shared<fgo::graph::GraphTimeCentricKimera>(*app_);
    
    if (!graph_) {
      app_->getLogger().error("KimeraIntegrationInterface: Failed to create GraphTimeCentricKimera");
      return false;
    }
    
    // Set Kimera-specific parameters (mirrors ImuParams structure from Kimera-VIO)
    fgo::graph::GraphTimeCentricKimeraParams kimera_params;
    kimera_params.timestampMatchTolerance = 0.001;
    kimera_params.createStatesAtIMURate = true;
    kimera_params.imuStateFrequency = params.imu_rate;
    
    // IMU factor configuration (pass through from ImuParams.yaml via adapter)
    // Convert int to enum type (mirrors VioBackend's use of imu_params_.imu_preintegration_type_)
    kimera_params.imuPreintegrationType = 
        static_cast<fgo::graph::ImuPreintegrationType>(params.imu_preintegration_type);
    kimera_params.accRandomWalk = params.accel_bias_rw_sigma;   // accelerometer_random_walk
    kimera_params.gyroRandomWalk = params.gyro_bias_rw_sigma;   // gyroscope_random_walk
    kimera_params.nominalSamplingTimeS = 1.0 / params.imu_rate; // 1/rate_hz
    
    // kimera_params.addGPMotionPriors = params.use_gp_priors;  // COMMENTED OUT FOR IMU ISOLATION TESTING
    kimera_params.addGPMotionPriors = false;  // Disabled for IMU testing
    kimera_params.optimizeOnKeyframe = params.optimize_on_keyframe;
    
    app_->getLogger().info("KimeraIntegrationInterface: IMU preintegration type = " + 
                           std::to_string(params.imu_preintegration_type) +
                           " (0=Combined, 1=Regular+BiasFactor)");
    
    graph_->setKimeraParams(kimera_params);
    
    // Initialize Kimera support
    if (!graph_->initializeKimeraSupport()) {
      app_->getLogger().error("KimeraIntegrationInterface: Failed to initialize Kimera support");
      return false;
    }
    
    initialized_ = true;
    app_->getLogger().info("KimeraIntegrationInterface: initialized successfully");
    
    return true;
    
  } catch (const std::exception& e) {
    app_->getLogger().error("KimeraIntegrationInterface: Exception during initialization: " + 
                            std::string(e.what()));
    return false;
  }
}

// ========================================================================
// STATE MANAGEMENT
// ========================================================================

StateHandle KimeraIntegrationInterface::createStateAtTimestamp(double timestamp) {
  if (!initialized_) {
    app_->getLogger().error("KimeraIntegrationInterface: Not initialized");
    return StateHandle();
  }
  
  std::cout << "[online_fgo_core] KimeraIntegrationInterface: CREATE STATE AT TIMESTAMP " << timestamp << std::endl;
  
  try {
    size_t state_idx = graph_->findOrCreateStateForTimestamp(timestamp, true);
    
    if (state_idx == 0) {
      app_->getLogger().error("KimeraIntegrationInterface: Failed to create state at timestamp " + 
                              std::to_string(timestamp));
      return StateHandle();
    }
    
    // Add to buffered timestamps for optimization
    bufferedStateTimestamps_.push_back(timestamp);
    
    app_->getLogger().debug("KimeraIntegrationInterface: Created state " + 
                            std::to_string(state_idx) + " at timestamp " + 
                            std::to_string(timestamp));
    
    return StateHandle(state_idx, timestamp);
    
  } catch (const std::exception& e) {
    app_->getLogger().error("KimeraIntegrationInterface: Exception creating state: " + 
                            std::string(e.what()));
    return StateHandle();
  }
}

StateHandle KimeraIntegrationInterface::createStateAtTimestamp(double timestamp,
                                                                const gtsam::Pose3& pose,
                                                                const gtsam::Vector3& velocity,
                                                                const gtsam::imuBias::ConstantBias& bias) {
  if (!initialized_) {
    app_->getLogger().error("KimeraIntegrationInterface: Not initialized");
    return StateHandle();
  }
  
  std::cout << "[online_fgo_core] KimeraIntegrationInterface: CREATE STATE AT TIMESTAMP " << timestamp 
            << " WITH INITIAL VALUES" << std::endl;
  
  try {
    // Create state (or find existing)
    size_t state_idx = graph_->findOrCreateStateForTimestamp(timestamp, true);
    
    if (state_idx == 0) {
      app_->getLogger().error("KimeraIntegrationInterface: Failed to create state at timestamp " + 
                              std::to_string(timestamp));
      return StateHandle();
    }
    
    // PRIORITY FIX #2: Set initial values from Kimera estimates
    // This ensures the state has proper initial estimates instead of relying on predicted buffer
    if (!graph_->setStateInitialValues(state_idx, pose, velocity, bias)) {
      app_->getLogger().warn("KimeraIntegrationInterface: Failed to set initial values for state " + 
                             std::to_string(state_idx) + ", using default values");
    } else {
      app_->getLogger().debug("KimeraIntegrationInterface: Set initial values for state " + 
                              std::to_string(state_idx));
    }
    
    // Add to buffered timestamps for optimization
    bufferedStateTimestamps_.push_back(timestamp);
    
    app_->getLogger().debug("KimeraIntegrationInterface: Created state " + 
                            std::to_string(state_idx) + " at timestamp " + 
                            std::to_string(timestamp) + " with initial values");
    
    return StateHandle(state_idx, timestamp);
    
  } catch (const std::exception& e) {
    app_->getLogger().error("KimeraIntegrationInterface: Exception creating state with initial values: " + 
                            std::string(e.what()));
    return StateHandle();
  }
}

StateHandle KimeraIntegrationInterface::getStateAtTimestamp(double timestamp) {
  if (!initialized_) {
    app_->getLogger().error("KimeraIntegrationInterface: Not initialized");
    return StateHandle();
  }
  
  if (!graph_->hasStateAtTimestamp(timestamp)) {
    app_->getLogger().warn("KimeraIntegrationInterface: No state at timestamp " + 
                           std::to_string(timestamp));
    return StateHandle();
  }
  
  size_t state_idx = graph_->getStateIndexAtTimestamp(timestamp);
  
  if (state_idx == 0) {
    return StateHandle();
  }
  
  return StateHandle(state_idx, timestamp);
}

std::vector<double> KimeraIntegrationInterface::getAllStateTimestamps() {
  if (!initialized_) {
    app_->getLogger().error("KimeraIntegrationInterface: Not initialized");
    return std::vector<double>();
  }
  
  return graph_->getAllStateTimestamps();
}


bool KimeraIntegrationInterface::addPreintegratedIMUData(
    const std::vector<std::pair<double, std::shared_ptr<gtsam::PreintegrationType>>>& pim_data) {
  if (!initialized_) {
    app_->getLogger().error("KimeraIntegrationInterface: Not initialized");
    return false;
  }
  
  if (pim_data.empty()) {
    app_->getLogger().warn("KimeraIntegrationInterface: Empty PIM data provided");
    return false;
  }
  
  std::cout << "[online_fgo_core] KimeraIntegrationInterface: ADD PREINTEGRATED IMU DATA, " 
            << pim_data.size() << " PIM values" << std::endl;
  
  try {
    // Forward to graph for storage and later use in factor construction
    return graph_->addPreintegratedIMUData(pim_data);
    
  } catch (const std::exception& e) {
    app_->getLogger().error("KimeraIntegrationInterface: Exception adding preintegrated IMU data: " + 
                            std::string(e.what()));
    return false;
  }
}

StateHandle KimeraIntegrationInterface::addKeyframeState(
    double timestamp,
    size_t frame_id,
    const gtsam::Pose3& pose,
    const gtsam::Vector3& velocity,
    const gtsam::imuBias::ConstantBias& bias) {
  if (!initialized_) {
    app_->getLogger().error("KimeraIntegrationInterface: Not initialized");
    return StateHandle();
  }

  std::cout << "[online_fgo_core] KimeraIntegrationInterface: ADD KEYFRAME STATE AT TIMESTAMP "
            << timestamp << " WITH FRAME_ID " << frame_id << std::endl;

  try {
    size_t state_idx = 0;
    bool success = graph_->addKeyframeState(timestamp, frame_id, pose, velocity, bias, state_idx);
    if (!success) {
      app_->getLogger().error("KimeraIntegrationInterface: Failed to add keyframe state at timestamp " +
                              std::to_string(timestamp));
      return StateHandle();
    }

    StateHandle handle(state_idx, timestamp);
    last_keyframe_handle_ = handle;
    return handle;
  } catch (const std::exception& e) {
    app_->getLogger().error("KimeraIntegrationInterface: Exception while adding keyframe state: " +
                            std::string(e.what()));
    return StateHandle();
  }
}

StateHandle KimeraIntegrationInterface::bootstrapInitialState(
    double timestamp,
    size_t frame_id,
    const gtsam::Pose3& pose,
    const gtsam::Vector3& velocity,
    const gtsam::imuBias::ConstantBias& bias) {
  if (!initialized_) {
    app_->getLogger().error("KimeraIntegrationInterface: Not initialized");
    return StateHandle();
  }

  app_->getLogger().info("KimeraIntegrationInterface: BOOTSTRAP INITIAL STATE at "
                         + std::to_string(timestamp) + " with frame_id " + std::to_string(frame_id));

  try {
    size_t state_idx = 0;
    bool success = graph_->bootstrapInitialState(timestamp, frame_id, pose, velocity, bias, state_idx);
    if (!success) {
      app_->getLogger().error("KimeraIntegrationInterface: Failed to bootstrap initial state");
      return StateHandle();
    }

    StateHandle handle(state_idx, timestamp);
    last_keyframe_handle_ = handle;
    bufferedStateTimestamps_.push_back(timestamp);
    return handle;
  } catch (const std::exception& e) {
    app_->getLogger().error("KimeraIntegrationInterface: Exception bootstrapping initial state: "
                            + std::string(e.what()));
    return StateHandle();
  }
}

bool KimeraIntegrationInterface::addImuFactorBetween(
    const StateHandle& previous_state,
    const StateHandle& current_state,
    std::shared_ptr<gtsam::PreintegrationType> pim) {
  if (!initialized_) {
    app_->getLogger().error("KimeraIntegrationInterface: Not initialized");
    return false;
  }

  if (!previous_state.valid || !current_state.valid) {
    app_->getLogger().error("KimeraIntegrationInterface: Invalid state handle(s) supplied for IMU factor");
    return false;
  }

  if (!pim) {
    app_->getLogger().error("KimeraIntegrationInterface: Null PIM provided for IMU factor");
    return false;
  }

  if (previous_state.index == current_state.index) {
    app_->getLogger().warn("KimeraIntegrationInterface: Skipping IMU factor between identical states");
    return false;
  }

  if (!graph_->addIMUFactorFromPIM(previous_state.index, current_state.index, *pim)) {
    app_->getLogger().warn("KimeraIntegrationInterface: Failed to add IMU factor between states " +
                           std::to_string(previous_state.index) + " and " +
                           std::to_string(current_state.index));
    return false;
  }

  app_->getLogger().debug("KimeraIntegrationInterface: Added IMU factor from state " +
                          std::to_string(previous_state.index) + " to " +
                          std::to_string(current_state.index));

  if (params_.optimize_on_keyframe) {
    app_->getLogger().info(
        "KimeraIntegrationInterface: IMU factor added; expect external smoother to consume incremental update");
  }

  return true;
}

// ========================================================================
// FACTOR INSERTION
// ========================================================================

bool KimeraIntegrationInterface::addFactor(const FactorData& factor_data) {
  if (!initialized_) {
    app_->getLogger().error("KimeraIntegrationInterface: Not initialized");
    return false;
  }
  
  // TODO: Implement when adding visual factors in Phase 3
  app_->getLogger().warn("KimeraIntegrationInterface: addFactor not yet implemented");
  return false;
}

bool KimeraIntegrationInterface::buildIncrementalUpdate(
    KimeraIntegrationInterface::IncrementalUpdatePacket* packet) {
  
  app_->getLogger().info("KimeraIntegrationInterface::buildIncrementalUpdate: ENTERED");
  
  if (!initialized_ || !graph_ || !packet) {
      app_->getLogger().error("KimeraIntegrationInterface::buildIncrementalUpdate: invalid state - initialized=" + 
                            std::to_string(initialized_) + " graph_=" + (graph_ ? "valid" : "null") + 
                            " packet=" + (packet ? "valid" : "null"));
    return false;
  }

  app_->getLogger().info("KimeraIntegrationInterface::buildIncrementalUpdate: calling graph_->buildIncrementalUpdate");
  
  bool result = graph_->buildIncrementalUpdate(
      &packet->factors, &packet->values, &packet->key_timestamps);
  
  
  
  app_->getLogger().info("KimeraIntegrationInterface::buildIncrementalUpdate: result=" + 
                        std::to_string(result) + " factors=" + std::to_string(packet->factors.size()) +
                        " values=" + std::to_string(packet->values.size()) +
                        " key_timestamps=" + std::to_string(packet->key_timestamps.size()));
  return result;
}

void KimeraIntegrationInterface::markIncrementalUpdateConsumed() {
  if (graph_) {
    graph_->finalizeIncrementalUpdate();
  }
}

// ========================================================================
// CONFIGURATION
// ========================================================================

void KimeraIntegrationInterface::updateParameters(const KimeraIntegrationParams& params) {
  params_ = params;
  
  if (graph_) {
    fgo::graph::GraphTimeCentricKimeraParams kimera_params;
    kimera_params.imuStateFrequency = params.imu_rate;
    kimera_params.addGPMotionPriors = params.use_gp_priors;
    kimera_params.optimizeOnKeyframe = params.optimize_on_keyframe;
    
    graph_->setKimeraParams(kimera_params);
    
    app_->getLogger().info("KimeraIntegrationInterface: Parameters updated");
  }
}


} // namespace fgo::integration
