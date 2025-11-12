//  Copyright 2022 Institute of Automatic Control RWTH Aachen University
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
//
//  ROS-agnostic port for online_fgo_core
//

#include <algorithm>
#include <sstream>
#include "online_fgo_core/graph/GraphBase.h"
#include "online_fgo_core/utils/AlgorithmicUtils.h"

namespace fgo::graph {
  using namespace std::chrono_literals;

  GraphBase::GraphBase(fgo::core::ApplicationInterface& app) :
    appPtr_(&app) {
    
    appPtr_->getLogger().info("---------------------  GraphBase initializing! --------------------- ");
    
    // Initialize parameter structure
    graphBaseParamPtr_ = std::make_shared<GraphParamBase>();
    
    // Initialize sensor calibration manager
    sensorCalibManager_ = std::make_shared<fgo::sensor::SensorCalibrationManager>(
        appPtr_->getLogger(), 
        appPtr_->getParameters(), 
        "GNSSFGO");
    
    // Initialize buffers
    referenceSensorTimestampBuffer_.buffer.set_capacity(50);
    referenceStateBuffer_.buffer.set_capacity(50);
    fgoOptStateBuffer_.buffer.set_capacity(50);
    accBuffer_.buffer.set_capacity(10000);
    currentPredictedBuffer_.buffer.set_capacity(50);

    // Load parameters
    graphBaseParamPtr_->publishResiduals = appPtr_->getParameters().getBool(
      "GNSSFGO.Graph.publishResiduals", true);
    appPtr_->getLogger().info("publishResiduals: " + 
        std::to_string(graphBaseParamPtr_->publishResiduals));

    graphBaseParamPtr_->publishResidualsOnline = appPtr_->getParameters().getBool(
        "GNSSFGO.Graph.publishResidualsOnline", true);
    appPtr_->getLogger().info("publishResidualsOnline: " + 
        std::to_string(graphBaseParamPtr_->publishResidualsOnline));

    if (graphBaseParamPtr_->publishResiduals) {
      graphBaseParamPtr_->onlyLastResiduals = appPtr_->getParameters().getBool(
          "GNSSFGO.Graph.onlyLastResiduals", true);
      appPtr_->getLogger().info("onlyLastResiduals: " + 
          std::to_string(graphBaseParamPtr_->onlyLastResiduals));

      factorBuffer_.buffer.set_capacity(100);
      resultMarginalBuffer_.buffer.set_capacity(100);
      
      // Create publishers using generic interface
      pubResiduals_ = appPtr_->createPublisher<FactorResiduals>("residuals");
      pubGTResiduals_ = appPtr_->createPublisher<FactorResiduals>("GTResiduals");
      
      // Load factor skipping configuration
      auto factorSkipped = appPtr_->getParameters().getStringArray(
          "GNSSFGO.Graph.factorSkipped", std::vector<std::string>{});

      for (const auto &name : factorSkipped) {
        try {
          graphBaseParamPtr_->skippedFactorsForResiduals.emplace_back(
              fgo::factor::FactorNameIDMap.at(name));
          appPtr_->getLogger().info("Skipping: " + name + " on residual publishing...");
        }
        catch (std::exception &ex) {
          appPtr_->getLogger().warn(name + " not available while skipping factors on residual publishing! " + 
              std::string(ex.what()));
        }
      }
      
      if (graphBaseParamPtr_->publishResidualsOnline) {
        resultMarginalBuffer_.buffer.set_capacity(100);
        shouldPublishResiduals_ = true;
        
        pubResidualsThread_ = std::make_shared<std::thread>(
          [this]() -> void {
            while (shouldPublishResiduals_) {
              const auto result_marginal_pairs = resultMarginalBuffer_.get_all_time_buffer_pair_and_clean();
              if (result_marginal_pairs.empty()) {
                std::this_thread::sleep_for(10ms);
                continue;
              }
              for (const auto &result_marginal_pair : result_marginal_pairs) {
                const auto &timestamp = result_marginal_pair.first;
                const auto &result = result_marginal_pair.second.first;
                const auto &marginals = result_marginal_pair.second.second;
                this->calculateResiduals(timestamp, result, marginals);
              }
            }
          }
        );
      }
    }

    // Smoother parameters
    graphBaseParamPtr_->smootherLag = appPtr_->getParameters().getDouble(
        "GNSSFGO.Optimizer.smootherLag", 0.1);
    appPtr_->getLogger().info("smootherLag: " + std::to_string(graphBaseParamPtr_->smootherLag));

    auto smootherType = appPtr_->getParameters().getString(
        "GNSSFGO.Optimizer.smootherType", "undefined");
    appPtr_->getLogger().info("smootherType: " + smootherType);

    if (smootherType == "BatchFixedLag") {
      graphBaseParamPtr_->smootherType = SmootherType::BatchFixedLag;
      
      // Levenberg-Marquardt parameters
      gtsam::LevenbergMarquardtParams lmp = gtsam::LevenbergMarquardtParams::LegacyDefaults();
      
      lmp.lambdaInitial = appPtr_->getParameters().getDouble(
          "Optimizer.LM.lambdaInitial", 1e-5);
      
      lmp.lambdaFactor = appPtr_->getParameters().getDouble(
          "Optimizer.LM.lambdaFactor", 10.0);
      
      lmp.lambdaUpperBound = appPtr_->getParameters().getDouble(
          "Optimizer.LM.lambdaUpperBound", 1e5);
      appPtr_->getLogger().info("lmp.lambdaUpperBound: " + std::to_string(lmp.lambdaUpperBound));
      
      lmp.lambdaLowerBound = appPtr_->getParameters().getDouble(
          "Optimizer.LM.lambdaLowerBound", 0.0);
      
      lmp.diagonalDamping = appPtr_->getParameters().getBool(
          "Optimizer.LM.diagonalDamping", false);
      
      lmp.useFixedLambdaFactor = appPtr_->getParameters().getBool(
          "Optimizer.LM.useFixedLambdaFactor", true);
      
      lmp.minDiagonal = appPtr_->getParameters().getDouble(
          "Optimizer.LM.minDiagonal", 1e-6);
      
      lmp.maxDiagonal = appPtr_->getParameters().getDouble(
          "Optimizer.LM.maxDiagonal", 1e32);
      
      this->initSolver(lmp);
      
    } else if (smootherType == "IncrementalFixedLag") {
      graphBaseParamPtr_->smootherType = SmootherType::ISAM2FixedLag;
      gtsam::ISAM2Params isam2Params;
      
      // Optimization method selection
      auto optimizationParams = appPtr_->getParameters().getString(
          "Optimizer.ISAM2.optimizationParams", "GN");
      if (optimizationParams == "DOGLEG") {
        isam2Params.optimizationParams = gtsam::ISAM2DoglegParams();
      } else {
        isam2Params.optimizationParams = gtsam::ISAM2GaussNewtonParams();
      }
      
      isam2Params.relinearizeThreshold = appPtr_->getParameters().getDouble(
          "Optimizer.ISAM2.relinearizeThreshold", 0.1);
      
      isam2Params.relinearizeSkip = appPtr_->getParameters().getInt(
          "Optimizer.ISAM2.relinearizeSkip", 10);
      
      auto factorization = appPtr_->getParameters().getString(
          "Optimizer.ISAM2.factorization", "CHOLESKY");
      if (factorization == "QR") {
        isam2Params.factorization = gtsam::ISAM2Params::QR;
      } else {
        isam2Params.factorization = gtsam::ISAM2Params::CHOLESKY;
      }
      
      isam2Params.enableDetailedResults = appPtr_->getParameters().getBool(
          "Optimizer.ISAM2.enableDetailedResults", false);
      
      isam2Params.findUnusedFactorSlots = appPtr_->getParameters().getBool(
          "Optimizer.ISAM2.findUnusedFactorSlots", false);
      
      this->initSolver(isam2Params);
      
    } else {
      appPtr_->getLogger().error("Smoothertype undefined. Please choose Smoothertype.");
      return;
    }

    // GP Motion Prior parameters
    double testDt = appPtr_->getParameters().getDouble("GNSSFGO.Optimizer.testDt", 0.1);

    // Load GP type parameter
    auto gpTypeStr = appPtr_->getParameters().getString("GNSSFGO.Optimizer.gpType", "WNOJ");
    // Note: gpType enum conversion would need to be handled based on GraphParamBase structure

    if (graphBaseParamPtr_->gpType == fgo::data::GPModelType::WNOJ) {
      auto QcGPMotionPriorFullVec = appPtr_->getParameters().getDoubleArray(
          "GNSSFGO.Optimizer.QcGPWNOJMotionPriorFull", std::vector<double>(6, 1000.0));
      graphBaseParamPtr_->QcGPMotionPriorFull = gtsam::Vector6(QcGPMotionPriorFullVec.data());
      
      std::ostringstream oss;
      oss << graphBaseParamPtr_->QcGPMotionPriorFull;
      appPtr_->getLogger().info("QcGPWNOJMotionPrior: " + oss.str());

      auto QcFullVec = appPtr_->getParameters().getDoubleArray(
          "GNSSFGO.Optimizer.QcGPWNOJInterpolatorFull", std::vector<double>(6, 1000.0));
      graphBaseParamPtr_->QcGPInterpolatorFull = gtsam::Vector6(QcFullVec.data());
      
      oss.str("");
      oss << graphBaseParamPtr_->QcGPInterpolatorFull;
      appPtr_->getLogger().info("QcGPWNOJInterpolator: " + oss.str());
      
      gtsam::SharedNoiseModel qcModel = gtsam::noiseModel::Diagonal::Variances(
          graphBaseParamPtr_->QcGPMotionPriorFull);
      const auto qc = gtsam::noiseModel::Gaussian::Covariance(
        fgo::utils::calcQ3<6>(fgo::utils::getQc(qcModel), testDt));

      std::cout << "#################################################################" << std::endl;
      std::cout << fgo::utils::getQc(qcModel) << std::endl;
      std::cout << "###" << std::endl;
      qc->print("Q3:");
      std::cout << "#################################################################" << std::endl;

    } else if (graphBaseParamPtr_->gpType == fgo::data::GPModelType::WNOA) {
      auto QcGPMotionPriorFullVec = appPtr_->getParameters().getDoubleArray(
          "GNSSFGO.Optimizer.QcGPWNOAMotionPriorFull", std::vector<double>(6, 1000.0));
      graphBaseParamPtr_->QcGPMotionPriorFull = gtsam::Vector6(QcGPMotionPriorFullVec.data());
      
      std::ostringstream oss;
      oss << graphBaseParamPtr_->QcGPMotionPriorFull;
      appPtr_->getLogger().info("QcGPWNOAMotionPrior: " + oss.str());

      auto QcFullVec = appPtr_->getParameters().getDoubleArray(
          "GNSSFGO.Optimizer.QcGPWNOAInterpolatorFull", std::vector<double>(6, 1000.0));
      graphBaseParamPtr_->QcGPInterpolatorFull = gtsam::Vector6(QcFullVec.data());
      
      oss.str("");
      oss << graphBaseParamPtr_->QcGPInterpolatorFull;
      appPtr_->getLogger().info("QcGPWNOAInterpolator: " + oss.str());
      
      gtsam::SharedNoiseModel qcModel = gtsam::noiseModel::Diagonal::Variances(
          graphBaseParamPtr_->QcGPMotionPriorFull);
      auto qc = gtsam::noiseModel::Gaussian::Covariance(
        fgo::utils::calcQ<6>(fgo::utils::getQc(qcModel), testDt));

      std::cout << "#################################################################" << std::endl;
      std::cout << fgo::utils::getQc(qcModel) << std::endl;
      std::cout << "###" << std::endl;
      qc->print("Q:");
      std::cout << "#################################################################" << std::endl;
      
    } else if (graphBaseParamPtr_->gpType == fgo::data::GPModelType::Singer) {
      auto QcGPMotionPriorFullVec = appPtr_->getParameters().getDoubleArray(
          "GNSSFGO.Optimizer.QcGPSingerMotionPriorFull", std::vector<double>(6, 1000.0));
      graphBaseParamPtr_->QcGPMotionPriorFull = gtsam::Vector6(QcGPMotionPriorFullVec.data());
      
      std::ostringstream oss;
      oss << graphBaseParamPtr_->QcGPMotionPriorFull;
      appPtr_->getLogger().info("QcGPSingerMotionPrior: " + oss.str());

      auto QcFullVec = appPtr_->getParameters().getDoubleArray(
          "GNSSFGO.Optimizer.QcGPSingerInterpolatorFull", std::vector<double>(6, 1000.0));
      graphBaseParamPtr_->QcGPInterpolatorFull = gtsam::Vector6(QcFullVec.data());
      
      oss.str("");
      oss << graphBaseParamPtr_->QcGPInterpolatorFull;
      appPtr_->getLogger().info("QcGPSingerInterpolator: " + oss.str());
    }

    // AutoDiff parameters
    graphBaseParamPtr_->AutoDiffNormalFactor = appPtr_->getParameters().getBool(
        "GNSSFGO.Graph.AutoDiffNormalFactor", false);
    appPtr_->getLogger().info("AutoDiffNormalFactor: " + 
        std::string(graphBaseParamPtr_->AutoDiffNormalFactor ? "true" : "false"));

    graphBaseParamPtr_->AutoDiffGPInterpolatedFactor = appPtr_->getParameters().getBool(
        "GNSSFGO.Graph.AutoDiffGPInterpolatedFactor", false);
    appPtr_->getLogger().info("AutoDiffGPInterpolatedFactor: " + 
        std::string(graphBaseParamPtr_->AutoDiffGPInterpolatedFactor ? "true" : "false"));

    graphBaseParamPtr_->AutoDiffGPMotionPriorFactor = appPtr_->getParameters().getBool(
        "GNSSFGO.Graph.AutoDiffGPMotionPriorFactor", false);
    appPtr_->getLogger().info("AutoDiffGPMotionPriorFactor: " + 
        std::string(graphBaseParamPtr_->AutoDiffGPMotionPriorFactor ? "true" : "false"));

    graphBaseParamPtr_->GPInterpolatedFactorCalcJacobian = appPtr_->getParameters().getBool(
        "GNSSFGO.Graph.GPInterpolatedFactorCalcJacobian", false);
    appPtr_->getLogger().info("GPInterpolatedFactorCalcJacobian: " + 
        std::string(graphBaseParamPtr_->GPInterpolatedFactorCalcJacobian ? "true" : "false"));

    graphBaseParamPtr_->addEstimatedVarianceAfterInit = appPtr_->getParameters().getBool(
        "GNSSFGO.Graph.addEstimatedVarianceAfterInit", false);
    appPtr_->getLogger().info("addEstimatedVarianceAfterInit: " + 
        std::string(graphBaseParamPtr_->addEstimatedVarianceAfterInit ? "true" : "false"));

    // Factor parameters
    graphBaseParamPtr_->addConstDriftFactor = appPtr_->getParameters().getBool(
        "GNSSFGO.Graph.addConstDriftFactor", true);
    appPtr_->getLogger().info("addConstDriftFactor: " + 
        std::string(graphBaseParamPtr_->addConstDriftFactor ? "true" : "false"));

    auto noiseModelClockFactor = appPtr_->getParameters().getString(
        "GNSSFGO.Graph.noiseModelClockFactor", "GAUSSIAN");
    // Note: setNoiseModelFromParam is an IntegratorBase static method - need to handle this
    // For now, set directly based on string
    if (noiseModelClockFactor == "CAUCHY") {
      graphBaseParamPtr_->noiseModelClockFactor = fgo::data::NoiseModel::CAUCHY;
    } else if (noiseModelClockFactor == "HUBER") {
      graphBaseParamPtr_->noiseModelClockFactor = fgo::data::NoiseModel::HUBER;
    } else {
      graphBaseParamPtr_->noiseModelClockFactor = fgo::data::NoiseModel::GAUSSIAN;
    }
    appPtr_->getLogger().info("clockDriftNoiseModel: " + noiseModelClockFactor);

    graphBaseParamPtr_->robustParamClockFactor = appPtr_->getParameters().getDouble(
        "GNSSFGO.Graph.robustParamClockFactor", 1.0);
    appPtr_->getLogger().info("robustParamClockFactor: " + 
        std::to_string(graphBaseParamPtr_->robustParamClockFactor));

    graphBaseParamPtr_->angularRateStd = appPtr_->getParameters().getDouble(
        "GNSSFGO.Graph.angularRateStd", 1.0);
    appPtr_->getLogger().info("angularRateStd: " + std::to_string(graphBaseParamPtr_->angularRateStd));

    graphBaseParamPtr_->constDriftStd = appPtr_->getParameters().getDouble(
        "GNSSFGO.Graph.constDriftStd", 1.0);
    appPtr_->getLogger().info("constDriftStd: " + std::to_string(graphBaseParamPtr_->constDriftStd));

    graphBaseParamPtr_->constBiasStd = appPtr_->getParameters().getDouble(
        "GNSSFGO.Graph.constBiasStd", 1.0);
    appPtr_->getLogger().info("constBiasStd: " + std::to_string(graphBaseParamPtr_->constBiasStd));

    graphBaseParamPtr_->motionModelStd = appPtr_->getParameters().getDouble(
        "GNSSFGO.Graph.motionModelStd", 1.0);

    double magnetometerStd = appPtr_->getParameters().getDouble(
        "GNSSFGO.Graph.magnetometerStd", 1.0);
    graphBaseParamPtr_->magnetometerStd = gtsam::Vector3(magnetometerStd, magnetometerStd, magnetometerStd);

    graphBaseParamPtr_->StateMeasSyncUpperBound = appPtr_->getParameters().getDouble(
        "GNSSFGO.Graph.StateMeasSyncUpperBound", 0.02);
    appPtr_->getLogger().info("StateMeasSyncUpperBound: " + 
        std::to_string(graphBaseParamPtr_->StateMeasSyncUpperBound));

    graphBaseParamPtr_->StateMeasSyncLowerBound = appPtr_->getParameters().getDouble(
        "GNSSFGO.Graph.StateMeasSyncLowerBound", -0.02);
    appPtr_->getLogger().info("StateMeasSyncLowerBound: " + 
        std::to_string(graphBaseParamPtr_->StateMeasSyncLowerBound));

    appPtr_->getLogger().info("---------------------  GraphBase initialized! --------------------- ");
  }

  void GraphBase::notifyOptimization() {
    // Notify application that optimization occurred
    // In the ROS version, this called appPtr_->notifyOptimization()
    // For now, this is a no-op in core - applications can override or use callbacks
  }

  void GraphBase::calculateResiduals(const fgo::core::TimeStamp &timestamp, 
                                     const gtsam::Values &result,
                                     const gtsam::Marginals &marginals) {
    uint64_t maxStateIndex = 0;

    for (const auto &key: result.keys()) {
      const auto keyIndex = gtsam::symbolIndex(key);
      if (keyIndex > maxStateIndex)
        maxStateIndex = keyIndex;
    }

    const auto factorBuffer = factorBuffer_.get_all_time_buffer_pair();

    for (const auto &factorVectorTimePair: factorBuffer) {
      if (factorVectorTimePair.first > timestamp)
        continue;
      
      FactorResiduals resMsg;
      resMsg.timestamp = timestamp.seconds();
      
      const auto &factorVec = factorVectorTimePair.second;
      std::for_each(factorVec.begin(), factorVec.end(),
                    [&](const gtsam::NonlinearFactor::shared_ptr &factor) -> void {
                      bool sampleResiduals = false;
                      bool factorImplemented = true;
                      gtsam::NoiseModelFactor::shared_ptr thisFactorCasted;
                      // Note: GTSAM factors don't have getTypeID(), using typeid as alternative
                      const auto factorTypeName = std::string(typeid(*factor).name());
                      
                      // Skip factors based on type name instead of ID
                      if (factorTypeName.find("GPWNOAPrior") == std::string::npos &&
                          factorTypeName.find("CombinedImuFactor") == std::string::npos &&
                          factorTypeName.find("ConstDriftFactor") == std::string::npos &&
                          factorTypeName.find("ConstAngularRateFactor") == std::string::npos) {
                        // Only process known factor types
                        return;
                      }
                      
                      // Process different factor types
                      if (factorTypeName.find("GPWNOAPrior") != std::string::npos) {
                        sampleResiduals = true;
                        thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::GPWNOAPrior>(factor);
                      } else if (factorTypeName.find("CombinedImuFactor") != std::string::npos) {
                        thisFactorCasted = boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(factor);
                      } else if (factorTypeName.find("ConstDriftFactor") != std::string::npos) {
                        thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::ConstDriftFactor>(factor);
                      } else if (factorTypeName.find("ConstAngularRateFactor") != std::string::npos) {
                        thisFactorCasted = boost::dynamic_pointer_cast<fgo::factor::ConstAngularRateFactor>(factor);
                      } else {
                        appPtr_->getLogger().error("onResidualPublishing factor " + 
                                                factorTypeName + " not implemented");
                        factorImplemented = false;
                      }

                      if (thisFactorCasted && factorImplemented) {
                        FactorResidual res;
                        res.factor_type = factorTypeName;  // Use type name instead of getName()
                        res.timestamp = timestamp.seconds();
                        
                        gtsam::Values values;
                        const gtsam::KeyVector keys = thisFactorCasted->keys();

                        bool allKeyValued = true;
                        std::for_each(keys.cbegin(), keys.cend(), [&](size_t key) -> void {
                          if (result.exists(key))
                            values.insert(key, result.at(key));
                          else
                            allKeyValued = false;
                        });

                        if (allKeyValued) {
                          const auto unwhitenedError = thisFactorCasted->unwhitenedError(values);
                          res.residuals.resize(unwhitenedError.size());
                          for (int i = 0; i < unwhitenedError.size(); ++i) {
                            res.residuals[i] = unwhitenedError(i);
                          }
                          resMsg.residuals.emplace_back(res);
                        }
                      }
                    });
      
      if (pubResiduals_ && !resMsg.residuals.empty()) {
        pubResiduals_->publish(resMsg);
      }
    }
  }

  void GraphBase::calculateGroundTruthResiduals(const fgo::core::TimeStamp &timestamp, 
                                                const gtsam::Values &gt) {
    // Ground truth residual calculation - currently not implemented
    // Can be added if ground truth data is available
  }

  void GraphBase::initGraph(const fgo::data::State &initState,
                           const double &initTimestamp,
                           boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> preIntegratorParams) {
    isStateInited_ = false;
    preIntegratorParams_ = preIntegratorParams;

    this->resetGraph();

    auto priorPoseFactor = gtsam::PriorFactor<gtsam::Pose3>(X(0), initState.state.pose(), initState.poseVar);
    this->push_back(priorPoseFactor);
    
    auto priorVelFactor = gtsam::PriorFactor<gtsam::Vector3>(V(0), initState.state.v(), initState.velVar);
    this->push_back(priorVelFactor);
    
    auto priorBiasFactor = gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), initState.imuBias,
                                                                            initState.imuBiasVar);
    this->push_back(priorBiasFactor);

    values_.insert(X(0), initState.state.pose());
    values_.insert(V(0), initState.state.v());
    values_.insert(B(0), initState.imuBias);

    keyTimestampMap_[X(0)] =
    keyTimestampMap_[V(0)] =
    keyTimestampMap_[B(0)] = initTimestamp;

    if (graphBaseParamPtr_->addConstDriftFactor) {
      auto priorClockErrorFactor = gtsam::PriorFactor<gtsam::Vector2>(C(0), initState.cbd, initState.cbdVar);
      this->push_back(priorClockErrorFactor);
      values_.insert(C(0), initState.cbd);
      keyTimestampMap_[C(0)] = initTimestamp;
    }

    currentKeyIndexTimestampMap_.insert(std::make_pair(nState_, initTimestamp));

    if (graphBaseParamPtr_->addGPPriorFactor || graphBaseParamPtr_->addGPInterpolatedFactor) {
      auto priorOmegaFactor = gtsam::PriorFactor<gtsam::Vector3>(W(0), initState.omega, initState.omegaVar);
      this->push_back(priorOmegaFactor);
      values_.insert(W(0), initState.omega);
      keyTimestampMap_[W(0)] = initTimestamp;
    }

    if (graphBaseParamPtr_->gpType == fgo::data::GPModelType::WNOJFull || 
        graphBaseParamPtr_->gpType == fgo::data::GPModelType::SingerFull || 
        graphBaseParamPtr_->addConstantAccelerationFactor) {
      auto priorAccFactor = gtsam::PriorFactor<gtsam::Vector6>(A(0), initState.accMeasured,
                                                               initState.accVar);
      this->push_back(priorAccFactor);
      values_.insert(A(0), gtsam::Vector6(initState.accMeasured));
      keyTimestampMap_[A(0)] = initTimestamp;
    }

    // Drop measurements before init timestamp for all integrators
    for (const auto &integrator: integratorMap_) {
      integrator.second->dropMeasurementBefore(initTimestamp);
    }

    isStateInited_ = true;
    appPtr_->getLogger().warn("------------- GraphBase: Init.FGO Done! -------------");
  }

} // namespace fgo::graph
