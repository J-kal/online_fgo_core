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
// Note: OmegaAtState is now defined in KimeraIntegrationInterface.h
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>   // for mkdir, stat, S_ISDIR
#include <sys/types.h> // for stat
#include <limits.h>    // for PATH_MAX
#include <stdlib.h>    // for realpath
#include <unistd.h>    // for getcwd

namespace fgo::integration {

// Helper function to create directory recursively
static bool createDirectoryRecursive(const std::string& path) {
  if (path.empty()) {
    return false;
  }
  
  // Check if directory already exists
  struct stat info;
  if (stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode)) {
    return true;  // Directory already exists
  }
  
  // Try to create the directory
  if (mkdir(path.c_str(), 0755) == 0) {
    return true;  // Successfully created
  }
  
  // If mkdir failed, try creating parent directories first
  size_t last_slash = path.find_last_of('/');
  if (last_slash != std::string::npos && last_slash > 0) {
    std::string parent = path.substr(0, last_slash);
    if (createDirectoryRecursive(parent)) {
      // Parent created, try again
      return mkdir(path.c_str(), 0755) == 0;
    }
  }
  
  return false;
}

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
    
    // GP motion prior configuration (Qc noise model is passed separately via setGPPriorParams)
    kimera_params.addGPMotionPriors = params.use_gp_priors;
    kimera_params.gpType = static_cast<fgo::data::GPModelType>(params.gp_model_type);
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

bool KimeraIntegrationInterface::setOmegaForState(size_t state_id, const OmegaAtState& omega_state) {
  if (!initialized_) {
    app_->getLogger().error("KimeraIntegrationInterface: Not initialized");
    return false;
  }
  
  if (!omega_state.valid) {
    app_->getLogger().warn("KimeraIntegrationInterface: Invalid OmegaAtState for state " + 
                           std::to_string(state_id));
    return false;
  }
  
  try {
    return graph_->setOmegaForState(state_id, omega_state);
  } catch (const std::exception& e) {
    app_->getLogger().error("KimeraIntegrationInterface: Exception setting omega for state " + 
                            std::to_string(state_id) + ": " + std::string(e.what()));
    return false;
  }
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

bool KimeraIntegrationInterface::addImuFactorWithOmega(
    const StateHandle& previous_state,
    const StateHandle& current_state,
    const gtsam::PreintegrationType& pim,
    const OmegaAtState& omega_from,
    const OmegaAtState& omega_to) {
  if (!initialized_) {
    app_->getLogger().error("KimeraIntegrationInterface: Not initialized");
    return false;
  }

  if (!previous_state.valid || !current_state.valid) {
    app_->getLogger().error("KimeraIntegrationInterface: Invalid state handle(s) supplied for IMU factor with omega");
    return false;
  }

  if (previous_state.index == current_state.index) {
    app_->getLogger().warn("KimeraIntegrationInterface: Skipping IMU factor between identical states");
    return false;
  }

  // 1. Add IMU factor (completely separate from GP priors)
  if (!graph_->addIMUFactorFromPIM(previous_state.index, current_state.index, pim)) {
    app_->getLogger().warn("KimeraIntegrationInterface: Failed to add IMU factor between states " +
                           std::to_string(previous_state.index) + " and " +
                           std::to_string(current_state.index));
    return false;
  }
  
  app_->getLogger().debug("KimeraIntegrationInterface: Added IMU factor from state " +
                          std::to_string(previous_state.index) + " to " +
                          std::to_string(current_state.index));

  // 2. Add GP motion prior (completely separate from IMU factor)
  if (params_.use_gp_priors && omega_from.valid && omega_to.valid) {
    const double dt = pim.deltaTij();
    
    if (!graph_->addGPMotionPrior(previous_state.index, current_state.index, 
                                   dt, omega_from, omega_to)) {
      app_->getLogger().warn("KimeraIntegrationInterface: Failed to add GP motion prior between states " +
                             std::to_string(previous_state.index) + " and " +
                             std::to_string(current_state.index) + " (continuing without GP prior)");
      // Don't fail - IMU factor was added successfully
    }
  }

  if (params_.optimize_on_keyframe) {
    app_->getLogger().info(
        "KimeraIntegrationInterface: IMU factor added; expect external smoother to consume incremental update");
  }

  return true;
}

bool KimeraIntegrationInterface::addGPMotionPrior(
    const StateHandle& previous_state,
    const StateHandle& current_state,
    double dt,
    const OmegaAtState& omega_from,
    const OmegaAtState& omega_to) {
  if (!initialized_) {
    app_->getLogger().error("KimeraIntegrationInterface: Not initialized");
    return false;
  }

  if (!previous_state.valid || !current_state.valid) {
    app_->getLogger().error("KimeraIntegrationInterface: Invalid state handle(s) for GP motion prior");
    return false;
  }

  if (!omega_from.valid || !omega_to.valid) {
    app_->getLogger().warn("KimeraIntegrationInterface: Invalid omega data for GP motion prior");
    return false;
  }

  if (!graph_->addGPMotionPrior(previous_state.index, current_state.index,
                                 dt, omega_from, omega_to)) {
    app_->getLogger().warn("KimeraIntegrationInterface: Failed to add GP motion prior between states " +
                           std::to_string(previous_state.index) + " and " +
                           std::to_string(current_state.index));
    return false;
  }

  app_->getLogger().debug("KimeraIntegrationInterface: Added GP motion prior from state " +
                          std::to_string(previous_state.index) + " to " +
                          std::to_string(current_state.index));

  return true;
}

bool KimeraIntegrationInterface::saveFactorGraphDebugInfo(int iteration, 
                                                          const std::string& context,
                                                          const std::string& save_dir) {
  if (!initialized_ || !graph_) {
    app_->getLogger().warn("KimeraIntegrationInterface: Cannot save factor graph - not initialized");
    return false;
  }
  
  try {
    // Get current graph and values
    gtsam::NonlinearFactorGraph current_graph;
    gtsam::Values current_values;
    
    if (!graph_->getCurrentGraphAndValues(current_graph, current_values)) {
      app_->getLogger().warn("KimeraIntegrationInterface: Failed to get current graph and values");
      return false;
    }
    
    // Create save directory recursively if it doesn't exist
    if (!createDirectoryRecursive(save_dir)) {
      app_->getLogger().warn("KimeraIntegrationInterface: Failed to create directory: " + save_dir + 
                             " (files may not be saved)");
      // Try to continue - maybe directory already exists
    }
    
    // Generate filename with iteration number
    std::ostringstream filename_ss;
    filename_ss << save_dir << "/factor_graph_" << context << "_iter" << iteration;
    std::string prefix = filename_ss.str();
    
    // Save .g2o file (standard factor graph format)
    std::string g2o_filename = prefix + ".g2o";
    
    // Get absolute path for logging (helps debug where files actually go)
    char abs_path[PATH_MAX];
    char* resolved = realpath(save_dir.c_str(), abs_path);
    std::string abs_dir = (resolved != nullptr) ? std::string(abs_path) : save_dir;
    
    // Get current working directory for debugging
    char cwd[PATH_MAX];
    std::string current_dir = (getcwd(cwd, sizeof(cwd)) != nullptr) ? std::string(cwd) : "unknown";
    
    try {
      current_graph.saveGraph(g2o_filename, current_values);
      
      // Verify file was actually created
      struct stat file_info;
      bool file_exists = (stat(g2o_filename.c_str(), &file_info) == 0);
      
      app_->getLogger().info("KimeraIntegrationInterface: Saved factor graph to " + g2o_filename + 
                             " (factors=" + std::to_string(current_graph.size()) + 
                             ", values=" + std::to_string(current_values.size()) + 
                             ", dir=" + save_dir + ", abs_dir=" + abs_dir + 
                             ", cwd=" + current_dir + ", file_exists=" + (file_exists ? "yes" : "no") + ")");
      
      if (!file_exists) {
        app_->getLogger().error("KimeraIntegrationInterface: File was not created! Check permissions and path.");
        return false;
      }
    } catch (const std::exception& e) {
      app_->getLogger().error("KimeraIntegrationInterface: Failed to save .g2o file: " + 
                              std::string(e.what()) + " (path=" + g2o_filename + ", cwd=" + current_dir + ")");
      return false;
    }
    
    // Save .dot file (GraphViz visualization format)
    std::string dot_filename = prefix + ".dot";
    std::ofstream dot_file(dot_filename);
    if (dot_file.is_open()) {
      dot_file << "digraph G {" << '\n';
      dot_file << "  // Factor Graph Visualization" << '\n';
      dot_file << "  // Context: " << context << '\n';
      dot_file << "  // Iteration: " << iteration << '\n';
      dot_file << "  // Total factors: " << current_graph.size() << '\n';
      dot_file << "  // Total values: " << current_values.size() << '\n';
      
      // Add nodes for values
      for (const auto& key_value : current_values) {
        dot_file << "  " << gtsam::DefaultKeyFormatter(key_value.key) 
                << " [shape=box];" << '\n';
      }
      
      // Add edges for factors
      for (size_t i = 0; i < current_graph.size(); ++i) {
        if (current_graph[i] && current_graph[i]->size() > 1) {
          auto keys = current_graph[i]->keys();
          for (size_t j = 1; j < keys.size(); ++j) {
            dot_file << "  " << gtsam::DefaultKeyFormatter(keys[0]) 
                    << " -> " << gtsam::DefaultKeyFormatter(keys[j])
                    << " [label=\"F" << i << "\"];" << '\n';
          }
        }
      }
      
      dot_file << "}" << '\n';
      dot_file.close();
      app_->getLogger().info("KimeraIntegrationInterface: Saved factor graph visualization to " + dot_filename);
    } else {
      app_->getLogger().warn("KimeraIntegrationInterface: Failed to open .dot file for writing");
    }
    
    return true;
    
  } catch (const std::exception& e) {
    app_->getLogger().error("KimeraIntegrationInterface: Failed to save factor graph debug info: " + 
                            std::string(e.what()));
    return false;
  }
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
    kimera_params.gpType = static_cast<fgo::data::GPModelType>(params.gp_model_type);
    kimera_params.optimizeOnKeyframe = params.optimize_on_keyframe;
    
    graph_->setKimeraParams(kimera_params);
    
    app_->getLogger().info("KimeraIntegrationInterface: Parameters updated");
  }
}

// ========================================================================
// VISUAL FACTOR INTERFACE
// ========================================================================

void KimeraIntegrationInterface::setStereoCalibration(
    const gtsam::Cal3_S2Stereo::shared_ptr& stereo_cal,
    const gtsam::Pose3& B_Pose_leftCam,
    const gtsam::SharedNoiseModel& smart_noise,
    const gtsam::SmartProjectionParams& smart_params) {
  
  if (!graph_) {
    app_->getLogger().error("KimeraIntegrationInterface: Graph not initialized, cannot set stereo calibration");
    return;
  }
  
  graph_->setStereoCalibration(stereo_cal, B_Pose_leftCam, smart_noise, smart_params);
  app_->getLogger().info("KimeraIntegrationInterface: Stereo calibration and smart factor params set");
}

void KimeraIntegrationInterface::setGPPriorParams(
    const gtsam::SharedNoiseModel& gp_qc_model,
    const gtsam::Matrix6& gp_ad_matrix,
    const gtsam::SharedNoiseModel& gp_acc_prior_noise) {
  if (!graph_) {
    app_->getLogger().error("KimeraIntegrationInterface: Graph not initialized, cannot set GP prior params");
    return;
  }
  
  graph_->setGPPriorParams(gp_qc_model, gp_ad_matrix, gp_acc_prior_noise);
  app_->getLogger().info("KimeraIntegrationInterface: GP motion prior params set (Qc, ad, acc_prior)");
}

size_t KimeraIntegrationInterface::addStereoMeasurements(
    FrameId frame_id,
    const std::vector<StereoMeasurement>& stereo_measurements) {
  
  if (!initialized_ || !graph_) {
    app_->getLogger().error("KimeraIntegrationInterface: Not initialized, cannot add stereo measurements");
    return 0;
  }
  
  size_t n_added = graph_->addStereoMeasurementsToGraph(frame_id, stereo_measurements);
  
  app_->getLogger().debug("KimeraIntegrationInterface: Added " + std::to_string(n_added) + 
                          " stereo measurements at frame " + std::to_string(frame_id));
  
  return n_added;
}


} // namespace fgo::integration
