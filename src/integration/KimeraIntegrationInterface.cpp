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
    
    // Set Kimera-specific parameters
    fgo::graph::GraphTimeCentricKimeraParams kimera_params;
    kimera_params.timestampMatchTolerance = 0.001;
    kimera_params.createStatesAtIMURate = true;
    kimera_params.imuStateFrequency = params.imu_rate;
    kimera_params.useCombinedIMUFactor = true;
    kimera_params.addGPMotionPriors = params.use_gp_priors;
    kimera_params.optimizeOnKeyframe = params.optimize_on_keyframe;
    
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

// ========================================================================
// IMU DATA HANDLING
// ========================================================================

bool KimeraIntegrationInterface::addIMUData(double timestamp, 
                                            const Eigen::Vector3d& accel, 
                                            const Eigen::Vector3d& gyro, 
                                            double dt) {
  if (!initialized_) {
    app_->getLogger().error("KimeraIntegrationInterface: Not initialized");
    return false;
  }
  
  std::cout << "[online_fgo_core] KimeraIntegrationInterface: ADD IMU DATA AT TIMESTAMP " << timestamp << std::endl;
  
  try {
    // Convert to fgo::data::IMUMeasurement
    fgo::data::IMUMeasurement imu_meas = convertToIMUMeasurement(timestamp, accel, gyro, dt);
    
    // Add to graph
    std::vector<fgo::data::IMUMeasurement> imu_vec = {imu_meas};
    return graph_->addIMUMeasurements(imu_vec);
    
  } catch (const std::exception& e) {
    app_->getLogger().error("KimeraIntegrationInterface: Exception adding IMU data: " + 
                            std::string(e.what()));
    return false;
  }
}

size_t KimeraIntegrationInterface::addIMUDataBatch(const std::vector<double>& timestamps,
                                                    const std::vector<Eigen::Vector3d>& accels,
                                                    const std::vector<Eigen::Vector3d>& gyros,
                                                    const std::vector<double>& dts) {
  if (!initialized_) {
    app_->getLogger().error("KimeraIntegrationInterface: Not initialized");
    return 0;
  }
  
  if (!timestamps.empty()) {
    std::cout << "[online_fgo_core] KimeraIntegrationInterface: ADD IMU DATA BATCH, " << timestamps.size() << " measurements, from " << timestamps.front() << " to " << timestamps.back() << std::endl;
  }
  
  if (timestamps.size() != accels.size() || 
      timestamps.size() != gyros.size() || 
      timestamps.size() != dts.size()) {
    app_->getLogger().error("KimeraIntegrationInterface: Inconsistent batch sizes");
    return 0;
  }
  
  try {
    std::vector<fgo::data::IMUMeasurement> imu_measurements;
    imu_measurements.reserve(timestamps.size());
    
    for (size_t i = 0; i < timestamps.size(); ++i) {
      imu_measurements.push_back(convertToIMUMeasurement(timestamps[i], accels[i], gyros[i], dts[i]));
    }
    
    if (graph_->addIMUMeasurements(imu_measurements)) {
      return timestamps.size();
    }
    
    return 0;
    
  } catch (const std::exception& e) {
    app_->getLogger().error("KimeraIntegrationInterface: Exception adding batch IMU data: " + 
                            std::string(e.what()));
    return 0;
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

// ========================================================================
// OPTIMIZATION
// ========================================================================

OptimizationResult KimeraIntegrationInterface::optimize() {
  if (!initialized_) {
    app_->getLogger().error("KimeraIntegrationInterface: Not initialized");
    OptimizationResult result;
    result.error_message = "Not initialized";
    return result;
  }
  
  std::cout << "[online_fgo_core] KimeraIntegrationInterface: OPTIMIZE" << std::endl;
  
  OptimizationResult result;
  
  try {
    app_->getLogger().info("KimeraIntegrationInterface: Starting optimization...");
    
    // Check if we have states to optimize
    if (bufferedStateTimestamps_.empty()) {
      app_->getLogger().warn("KimeraIntegrationInterface: No states buffered for optimization");
      result.error_message = "No states buffered";
      return result;
    }
    
    // Construct factor graph from buffered timestamps
    auto status = graph_->constructFactorGraphFromTimestamps(bufferedStateTimestamps_);
    
    if (status == fgo::graph::StatusGraphConstruction::FAILED) {
      app_->getLogger().error("KimeraIntegrationInterface: Factor graph construction failed");
      result.error_message = "Factor graph construction failed";
      return result;
    } else if (status == fgo::graph::StatusGraphConstruction::NO_OPTIMIZATION) {
        app_->getLogger().warn("KimeraIntegrationInterface: Skipping optimization as per graph construction status.");
        result.success = true;
        result.optimization_time_ms = 0;
        result.num_states = 0;
        return result;
    }
    
    // Run optimization
    fgo::data::State optimized_state;
    double opt_time = graph_->optimizeWithExternalFactors(optimized_state);
    
    // Package results
    result.success = true;
    result.optimization_time_ms = opt_time * 1000.0;
    result.num_states = bufferedStateTimestamps_.size();
    result.num_factors = graph_->size();
    
    result.latest_nav_state = optimized_state.state;
    result.latest_bias = optimized_state.imuBias;
    result.latest_covariance = optimized_state.poseVar;  // TODO: Combine with vel and bias
    
    // Clear buffered timestamps after successful optimization
    bufferedStateTimestamps_.clear();
    
    app_->getLogger().info("KimeraIntegrationInterface: Optimization successful in " + 
                           std::to_string(opt_time) + " seconds");
    
    return result;
    
  } catch (const std::exception& e) {
    app_->getLogger().error("KimeraIntegrationInterface: Exception during optimization: " + 
                            std::string(e.what()));
    
    // Save debug information: factor graph keys, .g2o and .dot files
    if (graph_) {
      app_->getLogger().error("KimeraIntegrationInterface: Saving factor graph debug information...");
      try {
        // Get the current factor graph and values from the graph object
        // GraphBase inherits from NonlinearFactorGraph, so we can cast
        const gtsam::NonlinearFactorGraph& current_graph = *graph_;
        const gtsam::Values& current_values = graph_->getValues();
        
        saveFactorGraphDebugInfo(current_graph, current_values, "online_fgo_exception", app_->getLogger());
      } catch (const std::exception& debug_e) {
        app_->getLogger().error("Failed to save debug info: " + std::string(debug_e.what()));
      }
    }
    
    result.error_message = std::string("Exception: ") + e.what();
    return result;
  }
}

bool KimeraIntegrationInterface::optimizeAndGetLatestState(gtsam::NavState& nav_state, 
                                                           gtsam::imuBias::ConstantBias& bias) {
  OptimizationResult result = optimize();
  
  if (result.success) {
    nav_state = result.latest_nav_state;
    bias = result.latest_bias;
    return true;
  }
  
  return false;
}

// ========================================================================
// RESULT RETRIEVAL
// ========================================================================

std::optional<gtsam::NavState> KimeraIntegrationInterface::getOptimizedState(StateHandle handle) {
  if (!initialized_ || !handle.valid) {
    return std::nullopt;
  }
  
  auto pose = graph_->getOptimizedPose(handle.index);
  auto velocity = graph_->getOptimizedVelocity(handle.index);
  
  if (pose && velocity) {
    return gtsam::NavState(*pose, *velocity);
  }
  
  return std::nullopt;
}

std::optional<gtsam::imuBias::ConstantBias> KimeraIntegrationInterface::getOptimizedBias(
    StateHandle handle) {
  if (!initialized_ || !handle.valid) {
    return std::nullopt;
  }
  
  return graph_->getOptimizedBias(handle.index);
}

std::optional<gtsam::Matrix> KimeraIntegrationInterface::getStateCovariance(StateHandle handle) {
  if (!initialized_ || !handle.valid) {
    return std::nullopt;
  }
  
  return graph_->getStateCovariance(handle.index);
}

std::optional<gtsam::NavState> KimeraIntegrationInterface::getLatestOptimizedState() {
  if (!initialized_) {
    return std::nullopt;
  }
  
  auto timestamps = graph_->getAllStateTimestamps();
  
  if (timestamps.empty()) {
    app_->getLogger().warn("KimeraIntegrationInterface: No states available");
    return std::nullopt;
  }
  
  // Get latest timestamp
  double latest_ts = timestamps.back();
  size_t latest_idx = graph_->getStateIndexAtTimestamp(latest_ts);
  
  if (latest_idx == 0) {
    return std::nullopt;
  }
  
  StateHandle handle(latest_idx, latest_ts);
  return getOptimizedState(handle);
}

std::optional<gtsam::imuBias::ConstantBias> KimeraIntegrationInterface::getLatestOptimizedBias() {
  if (!initialized_) {
    return std::nullopt;
  }
  
  auto timestamps = graph_->getAllStateTimestamps();
  
  if (timestamps.empty()) {
    return std::nullopt;
  }
  
  double latest_ts = timestamps.back();
  size_t latest_idx = graph_->getStateIndexAtTimestamp(latest_ts);
  
  if (latest_idx == 0) {
    return std::nullopt;
  }
  
  StateHandle handle(latest_idx, latest_ts);
  return getOptimizedBias(handle);
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

// ========================================================================
// HELPER METHODS
// ========================================================================

fgo::data::IMUMeasurement KimeraIntegrationInterface::convertToIMUMeasurement(
    double timestamp,
    const Eigen::Vector3d& accel,
    const Eigen::Vector3d& gyro,
    double dt) {
  
  fgo::data::IMUMeasurement imu_meas;
  
  // Set timestamp
  imu_meas.timestamp = fgo::core::TimeStamp(timestamp);
  
  // Set linear acceleration
  imu_meas.accLin = gtsam::Vector3(accel.x(), accel.y(), accel.z());
  
  // Set gyro
  imu_meas.gyro = gtsam::Vector3(gyro.x(), gyro.y(), gyro.z());
  
  // Set dt
  imu_meas.dt = dt;
  
  // Set rotational acceleration (optional, zero for now)
  imu_meas.accRot = gtsam::Vector3::Zero();
  
  // Set covariances from parameters
  gtsam::Matrix3 accel_cov = gtsam::Matrix3::Identity() * 
                             (params_.accel_noise_sigma * params_.accel_noise_sigma);
  gtsam::Matrix3 gyro_cov = gtsam::Matrix3::Identity() * 
                            (params_.gyro_noise_sigma * params_.gyro_noise_sigma);
  
  imu_meas.accLinCov = accel_cov;
  imu_meas.gyroCov = gyro_cov;
  imu_meas.accRotCov = gtsam::Matrix3::Identity() * 1e-6;  // Small default
  
  return imu_meas;
}

} // namespace fgo::integration
