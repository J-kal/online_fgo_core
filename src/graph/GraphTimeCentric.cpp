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

#include "online_fgo_core/graph/GraphTimeCentric.h"
#include "online_fgo_core/graph/GraphUtils.h"

using namespace fgo::data;

namespace fgo::graph {

  GraphTimeCentric::GraphTimeCentric(fgo::core::ApplicationInterface& app) : GraphBase(app) {
    appPtr_->getLogger().info("GraphTimeCentric: initializing...");
    
    // Create publisher for IMU factor report
    pubIMUFactorReport_ = appPtr_->createPublisher<SensorProcessingReport>("imu_factor_report");
    
    appPtr_->getLogger().info("GraphTimeCentric: initialized!");
  }

  StatusGraphConstruction GraphTimeCentric::constructFactorGraphOnIMU(
      std::vector<fgo::data::IMUMeasurement>& dataIMU) {

    static boost::circular_buffer<std::pair<double, gtsam::Vector3>> timeGyroMap(
        20 * graphBaseParamPtr_->IMUMeasurementFrequency / graphBaseParamPtr_->optFrequency);
    static boost::circular_buffer<std::pair<size_t, gtsam::Vector6>> stateIDAccMap(
        50 * graphBaseParamPtr_->smootherLag * graphBaseParamPtr_->optFrequency);
    static gtsam::Vector6 lastAcc = gtsam::Vector6::Zero();
    static gtsam::Vector3 meanAccA, meanAccG = gtsam::Vector3();
    static uint64_t lastNState = 0;
    static bool skippedOpt = false;

    gtsam::Key pose_key_j, vel_key_j, bias_key_j, cbd_key_j, omega_key_j, acc_key_j,
        pose_key_i, vel_key_i, bias_key_i, cbd_key_i, omega_key_i, acc_key_i;

    bool hasMeasurements = true;
    double voteZeroVelocity = 0;
    for (const auto &integrator: integratorMap_) {
      hasMeasurements &= integrator.second->checkHasMeasurements();
      if (integrator.second->checkZeroVelocity()) {
        appPtr_->getLogger().warn("constructFactorGraphOnIMU: " + integrator.first + " reported ZERO VELOCITY");
        voteZeroVelocity++;
      }
    }

    static double noOptimizationDuration = 0.;
    static fgo::core::TimeStamp firstNoOptimizationDecision;
    if (paramPtr_->NoOptimizationNearZeroVelocity && (nState_ - lastNState) > paramPtr_->NoOptimizationAfterStates) {
      if (voteZeroVelocity / integratorMap_.size() > paramPtr_->VoteNearZeroVelocity) {
        appPtr_->getLogger().error("constructFactorGraphOnIMU: Near zero velocity with " + 
                               std::to_string(voteZeroVelocity) + " integrator of " + 
                               std::to_string(integratorMap_.size()) + " not optimizing...");

        if (!skippedOpt)
          firstNoOptimizationDecision = appPtr_->now();
        noOptimizationDuration = (appPtr_->now() - firstNoOptimizationDecision).seconds();
        
        for (const auto &integrator: integratorMap_) {
          integrator.second->cleanBuffers();
        }
        
        appPtr_->getLogger().error("constructFactorGraphOnIMU: Near zero velocity duration " + 
                               std::to_string(noOptimizationDuration));

        skippedOpt = true;
        solver_->setNotMarginalizing();
        return StatusGraphConstruction::NO_OPTIMIZATION;
      }
    }

    auto currentPredState = currentPredictedBuffer_.get_last_buffer();

    if (skippedOpt) {
      skippedOpt = false;
      lastNState = nState_;
      solver_->setSmootherLagInflation(noOptimizationDuration);

      for (const auto &integrator: integratorMap_) {
        integrator.second->notifyOptimization(noOptimizationDuration);
      }
      noOptimizationDuration = 0.;
    }

    if (paramPtr_->NoOptimizationWhileNoMeasurement && !hasMeasurements &&
        nState_ > paramPtr_->NoOptimizationAfterStates) {
      appPtr_->getLogger().error("constructFactorGraphOnIMU: NO Reference Measurement, not optimizing...");
      return StatusGraphConstruction::NO_OPTIMIZATION;
    }

    if (paramPtr_->calibGravity)
      preIntegratorParams_->n_gravity = fgo::utils::gravity_ecef(currentPredState.state.position());

    std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imuPreIntegrationOPT_ =
        std::make_shared<gtsam::PreintegratedCombinedMeasurements>(preIntegratorParams_,
                                                                   currentPredState.imuBias);

    double sum_imu_dt = 0.;
    size_t counter_imu = 0;
    auto currentStateTimestamp = dataIMU.front().timestamp.seconds();
    fgo::core::TimeStamp lastMeasTimestamp;

    for (const auto &thisIMU: dataIMU) {
      if ((thisIMU.timestamp.seconds() - currentStateTimestamp) < 
          0.001) {  // Use default threshold since parameter is missing
        timeGyroMap.push_back(std::make_pair(thisIMU.timestamp.seconds(), thisIMU.gyro));
        imuPreIntegrationOPT_->integrateMeasurement(thisIMU.accLin,
                                                    thisIMU.gyro,
                                                    thisIMU.dt);
        meanAccG += thisIMU.accRot;
        meanAccA += thisIMU.accLin;
        sum_imu_dt += thisIMU.dt;
        counter_imu++;
        lastMeasTimestamp = thisIMU.timestamp;
      } else {
        meanAccG /= static_cast<double>(counter_imu);
        meanAccA /= static_cast<double>(counter_imu);

        nState_++;
        pose_key_j = X(nState_);
        vel_key_j = V(nState_);
        bias_key_j = B(nState_);
        omega_key_j = W(nState_);
        acc_key_j = A(nState_);

        pose_key_i = X(nState_ - 1);
        vel_key_i = V(nState_ - 1);
        bias_key_i = B(nState_ - 1);
        omega_key_i = W(nState_ - 1);
        acc_key_i = A(nState_ - 1);

        auto predictedIMUState = imuPreIntegrationOPT_->predict(currentPredState.state,
                                                                 currentPredState.imuBias);

        gtsam::Vector3 gravity_b = gtsam::Vector3::Zero();
        if (paramPtr_->calibGravity) {
          const auto gravity = fgo::utils::gravity_ecef(currentPredState.state.position());
          gravity_b = currentPredState.state.attitude().unrotate(gravity);
        }

        auto currentAcc = (gtsam::Vector6() << meanAccG,
            currentPredState.imuBias.correctAccelerometer(meanAccA + gravity_b)).finished();
        
        stateIDAccMap.push_back(std::make_pair(nState_, currentAcc));
        accBuffer_.update_buffer(currentAcc, lastMeasTimestamp);
        meanAccG.setZero();
        meanAccA.setZero();

        appPtr_->getLogger().info("FGOnIMU: creating state variable " + std::to_string(nState_) + 
                              " at: " + std::to_string(currentStateTimestamp) + 
                              " IMU DT: " + std::to_string(sum_imu_dt));

        currentKeyIndexTimestampMap_.insert(std::make_pair(nState_, currentStateTimestamp));

        currentPredState.state = graph::queryCurrentPredictedState(
            currentPredictedBuffer_.get_all_time_buffer_pair(), currentStateTimestamp).state;

        values_.insert(pose_key_j, currentPredState.state.pose());
        values_.insert(vel_key_j, currentPredState.state.velocity());
        values_.insert(bias_key_j, currentPredState.imuBias);

        keyTimestampMap_[pose_key_j] =
        keyTimestampMap_[vel_key_j] =
        keyTimestampMap_[bias_key_j] = currentStateTimestamp;

        // IMU Factor
        boost::shared_ptr<gtsam::CombinedImuFactor> imu_factor =
            boost::make_shared<gtsam::CombinedImuFactor>(pose_key_i, vel_key_i,
                                                         pose_key_j, vel_key_j,
                                                         bias_key_i, bias_key_j,
                                                         *imuPreIntegrationOPT_);
        this->push_back(imu_factor);

        // Publish sensor processing report
        SensorProcessingReport thisProcessingReport;
        auto currentTime = appPtr_->now();
        thisProcessingReport.timestamp = currentTime.seconds();
        thisProcessingReport.sensor_name = "IMUPreIntegrated";
        thisProcessingReport.measurements_processed = counter_imu;
        thisProcessingReport.processing_time_ms = sum_imu_dt * 1000.0;

        if (pubIMUFactorReport_)
          pubIMUFactorReport_->publish(thisProcessingReport);

        // Const GNSS Clock error factor
        if (paramPtr_->addConstDriftFactor) {
          cbd_key_j = C(nState_);
          cbd_key_i = C(nState_ - 1);
          values_.insert(cbd_key_j, currentPredState.cbd);
          keyTimestampMap_[cbd_key_j] = currentStateTimestamp;
          this->addConstDriftFactor(cbd_key_i, cbd_key_j, sum_imu_dt, currentPredState.cbdVar.diagonal());
        }

        // Motion model factor
        if (paramPtr_->useMMFactor) {
          appPtr_->getLogger().info("FGConIMU: state variable " + std::to_string(nState_) + " with MM factor.");
          this->addMotionModelFactor(pose_key_i, vel_key_i, pose_key_j, vel_key_j, sum_imu_dt);
        }

        // GP prior
        if (paramPtr_->addGPPriorFactor) {
          gtsam::Matrix6 ad = gtsam::Matrix6::Identity();
          this->addGPMotionPrior(
              pose_key_i, vel_key_i, omega_key_i, acc_key_i,
              pose_key_j, vel_key_j, omega_key_j, acc_key_j, sum_imu_dt,
              lastAcc, currentAcc, ad);
          lastAcc = currentAcc;
        }

        if (paramPtr_->gpType == WNOJFull || paramPtr_->gpType == SingerFull ||
            paramPtr_->addConstantAccelerationFactor) {
          values_.insert(acc_key_j, currentAcc);
          keyTimestampMap_[acc_key_j] = currentStateTimestamp;
        }

        if (paramPtr_->addGPPriorFactor || paramPtr_->addGPInterpolatedFactor) {
          keyTimestampMap_[omega_key_j] = currentStateTimestamp;
          values_.insert(omega_key_j, currentPredState.omega);
        }
        
        // Reset for next iteration
        sum_imu_dt = 0.;
        counter_imu = 0;
        imuPreIntegrationOPT_->resetIntegration();
      }
    }

    /*
     *     Integrating sensor
     */

    bool integrationSuccessfully = true;

    auto statePair = currentPredictedBuffer_.get_all_time_buffer_pair();
    for (const auto &integrator: integratorMap_) {
      appPtr_->getLogger().info("GraphTimeCentric: starting integrating measurement from " + integrator.first);

      integrationSuccessfully &= integrator.second->addFactors(timeGyroMap,
                                                               stateIDAccMap,
                                                               currentKeyIndexTimestampMap_,
                                                               statePair,
                                                               values_,
                                                               keyTimestampMap_,
                                                               relatedKeys_);
      appPtr_->getLogger().info("GraphTimeCentric: integrating measurement from " + integrator.first + " was " +
                            (integrationSuccessfully ? "successful" : "failed!"));
    }

    if (paramPtr_->verbose)
      this->print("GraphTimeCentric: ");

    if (integrationSuccessfully)
      return StatusGraphConstruction::SUCCESSFUL;
    else
      return StatusGraphConstruction::FAILED;
  }

  StatusGraphConstruction GraphTimeCentric::constructFactorGraphOnTime(
      const std::vector<double> &stateTimestamps,
      std::vector<fgo::data::IMUMeasurement> &dataIMU) {

    static const double betweenOptimizationTime = 1. / paramPtr_->optFrequency;
    static boost::circular_buffer<std::pair<double, gtsam::Vector3>> timeGyroMap(
        20 * graphBaseParamPtr_->IMUMeasurementFrequency / graphBaseParamPtr_->optFrequency);
    static boost::circular_buffer<std::pair<size_t, gtsam::Vector6>> stateIDAccMap(
        50 * graphBaseParamPtr_->smootherLag * graphBaseParamPtr_->optFrequency);
    static gtsam::Vector6 lastAcc = gtsam::Vector6::Zero();
    static gtsam::Vector3 meanAccA, meanAccG = gtsam::Vector3();
    static uint64_t lastNState = 0;
    static bool skippedOpt = false;

    gtsam::Key pose_key_j, vel_key_j, bias_key_j, cbd_key_j, omega_key_j, acc_key_j,
        pose_key_i, vel_key_i, bias_key_i, cbd_key_i, omega_key_i, acc_key_i;

    bool hasMeasurements = true;
    double voteZeroVelocity = 0;
    for (const auto &integrator: integratorMap_) {
      hasMeasurements &= integrator.second->checkHasMeasurements();
      if (integrator.second->checkZeroVelocity()) {
        appPtr_->getLogger().warn("constructFactorGraphOnTime: " + integrator.first + 
                              " reported ZERO VELOCITY");
        voteZeroVelocity++;
      }
    }

    static double noOptimizationDuration = 0.;
    static fgo::core::TimeStamp firstNoOptimizationDecision;
    if (paramPtr_->NoOptimizationNearZeroVelocity && (nState_ - lastNState) > paramPtr_->NoOptimizationAfterStates) {
      if (voteZeroVelocity / integratorMap_.size() > paramPtr_->VoteNearZeroVelocity) {
        appPtr_->getLogger().error("constructFactorGraphOnTime: Near zero velocity with " + 
                               std::to_string(voteZeroVelocity) + " integrator of " + 
                               std::to_string(integratorMap_.size()) + " not optimizing...");

        if (!skippedOpt)
          firstNoOptimizationDecision = appPtr_->now();
        noOptimizationDuration = (appPtr_->now() - firstNoOptimizationDecision).seconds();
        
        for (const auto &integrator: integratorMap_) {
          integrator.second->cleanBuffers();
        }
        
        appPtr_->getLogger().error("constructFactorGraphOnTime: Near zero velocity duration " + 
                               std::to_string(noOptimizationDuration));

        skippedOpt = true;
        solver_->setNotMarginalizing();
        return StatusGraphConstruction::NO_OPTIMIZATION;
      }
    }

    auto currentPredState = currentPredictedBuffer_.get_last_buffer();

    if (skippedOpt) {
      skippedOpt = false;
      lastNState = nState_;
      solver_->setSmootherLagInflation(noOptimizationDuration);

      for (const auto &integrator: integratorMap_) {
        integrator.second->notifyOptimization(noOptimizationDuration);
      }
      noOptimizationDuration = 0.;
    }

    if (paramPtr_->NoOptimizationWhileNoMeasurement && !hasMeasurements &&
        nState_ > paramPtr_->NoOptimizationAfterStates) {
      appPtr_->getLogger().error("constructFactorGraphOnTime: NO Reference Measurement, not optimizing...");
      return StatusGraphConstruction::NO_OPTIMIZATION;
    }

    if (!dataIMURest_.empty()) {
      dataIMU.insert(dataIMU.begin(), dataIMURest_.begin(), dataIMURest_.end());
      dataIMURest_.clear();
    }

    if (paramPtr_->calibGravity)
      preIntegratorParams_->n_gravity = fgo::utils::gravity_ecef(currentPredState.state.position());

    std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imuPreIntegrationOPT_ =
        std::make_shared<gtsam::PreintegratedCombinedMeasurements>(preIntegratorParams_,
                                                                   currentPredState.imuBias);

    auto imuIter = dataIMU.begin();
    for (const auto &ts: stateTimestamps) {
      nState_++;
      pose_key_j = X(nState_);
      vel_key_j = V(nState_);
      bias_key_j = B(nState_);
      omega_key_j = W(nState_);
      acc_key_j = A(nState_);

      pose_key_i = X(nState_ - 1);
      vel_key_i = V(nState_ - 1);
      bias_key_i = B(nState_ - 1);
      omega_key_i = W(nState_ - 1);
      acc_key_i = A(nState_ - 1);

      double imuCounter = 0;
      auto currentIMU = dataIMU.back();
      while (imuIter != dataIMU.end() && imuIter->timestamp.seconds() < ts) {
        currentIMU = *imuIter;
        timeGyroMap.push_back(std::make_pair(currentIMU.timestamp.seconds(), currentIMU.gyro));
        imuPreIntegrationOPT_->integrateMeasurement(currentIMU.accLin,
                                                    currentIMU.gyro,
                                                    currentIMU.dt);
        meanAccG += currentIMU.accRot;
        imuCounter += 1.;
        imuIter = dataIMU.erase(imuIter);
      }

      meanAccG /= imuCounter;

      currentPredState = graph::queryCurrentPredictedState(
          currentPredictedBuffer_.get_all_time_buffer_pair(), ts);

      gtsam::Vector3 gravity_b = gtsam::Vector3::Zero();
      if (paramPtr_->calibGravity) {
        const auto gravity = fgo::utils::gravity_ecef(currentPredState.state.position());
        gravity_b = currentPredState.state.attitude().unrotate(gravity);
      }

      auto currentAcc = (gtsam::Vector6() << currentIMU.accRot,
          currentPredState.imuBias.correctAccelerometer(currentIMU.accLin + gravity_b)).finished();
      
      stateIDAccMap.push_back(std::make_pair(nState_, currentAcc));
      accBuffer_.update_buffer(currentAcc, currentIMU.timestamp);
      meanAccG.setZero();

      appPtr_->getLogger().info("constructFactorGraphOnTime: creating state variable " + 
                            std::to_string(nState_) + " at: " + std::to_string(ts) + 
                            " current IMU time " + std::to_string(currentIMU.timestamp.seconds()) + 
                            " queried predicted state at " + 
                            std::to_string(currentPredState.timestamp.seconds()));

      currentKeyIndexTimestampMap_.insert(std::make_pair(nState_, ts));

      // Setup values
      values_.insert(pose_key_j, currentPredState.state.pose());
      values_.insert(vel_key_j, currentPredState.state.velocity());
      values_.insert(bias_key_j, currentPredState.imuBias);

      keyTimestampMap_[pose_key_j] =
      keyTimestampMap_[vel_key_j] =
      keyTimestampMap_[bias_key_j] = ts;

      // IMU Factor
      boost::shared_ptr<gtsam::CombinedImuFactor> imu_factor =
          boost::make_shared<gtsam::CombinedImuFactor>(pose_key_i, vel_key_i,
                                                       pose_key_j, vel_key_j,
                                                       bias_key_i, bias_key_j,
                                                       *imuPreIntegrationOPT_);
      this->push_back(imu_factor);

      // Publish sensor processing report
      SensorProcessingReport thisProcessingReport;
      auto currentTime = appPtr_->now();
      thisProcessingReport.timestamp = currentTime.seconds();
      thisProcessingReport.sensor_name = "IMUPreIntegrated";
      thisProcessingReport.measurements_processed = imuCounter;
      thisProcessingReport.processing_time_ms = betweenOptimizationTime * 1000.0;

      if (pubIMUFactorReport_)
        pubIMUFactorReport_->publish(thisProcessingReport);

      // Const GNSS Clock error factor
      if (paramPtr_->addConstDriftFactor) {
        cbd_key_j = C(nState_);
        cbd_key_i = C(nState_ - 1);
        values_.insert(cbd_key_j, currentPredState.cbd);
        keyTimestampMap_[cbd_key_j] = ts;
        this->addConstDriftFactor(cbd_key_i, cbd_key_j, betweenOptimizationTime, 
                                 currentPredState.cbdVar.diagonal());
      }

      // Motion model factor
      if (paramPtr_->useMMFactor) {
        appPtr_->getLogger().info("constructFactorGraphOnTime: state variable " + 
                              std::to_string(nState_) + " with MM factor.");
        this->addMotionModelFactor(pose_key_i, vel_key_i, pose_key_j, vel_key_j, betweenOptimizationTime);
      }

      // GP prior
      if (paramPtr_->addGPPriorFactor) {
        gtsam::Matrix6 ad = gtsam::Matrix6::Identity();
        this->addGPMotionPrior(
            pose_key_i, vel_key_i, omega_key_i, acc_key_i,
            pose_key_j, vel_key_j, omega_key_j, acc_key_j, betweenOptimizationTime,
            lastAcc, currentAcc, ad);
        lastAcc = currentAcc;
      }

      if (paramPtr_->gpType == WNOJFull || paramPtr_->gpType == SingerFull ||
          paramPtr_->addConstantAccelerationFactor) {
        values_.insert(acc_key_j, currentAcc);
        keyTimestampMap_[acc_key_j] = ts;
      }

      if (paramPtr_->addGPPriorFactor || paramPtr_->addGPInterpolatedFactor) {
        keyTimestampMap_[omega_key_j] = ts;
        values_.insert(omega_key_j, currentPredState.omega);
      }
      
      imuPreIntegrationOPT_->resetIntegration();
    }

    // Backup leftover IMU measurements
    if (!dataIMU.empty()) {
      appPtr_->getLogger().info("constructFactorGraphOnTime: " + std::to_string(dataIMU.size()) + 
                            " imu measurements are left. Backing up...");
      dataIMURest_.resize(dataIMU.size());
      std::copy(dataIMU.begin(), dataIMU.end(), dataIMURest_.begin());
    }

    /*
     *     Integrating sensor
     */

    bool integrationSuccessfully = true;

    auto statePair = currentPredictedBuffer_.get_all_time_buffer_pair();
    for (const auto &integrator: integratorMap_) {
      appPtr_->getLogger().info("GraphTimeCentric: starting integrating measurement from " + integrator.first);

      integrationSuccessfully &= integrator.second->addFactors(timeGyroMap,
                                                               stateIDAccMap,
                                                               currentKeyIndexTimestampMap_,
                                                               statePair,
                                                               values_,
                                                               keyTimestampMap_,
                                                               relatedKeys_);
      appPtr_->getLogger().info("GraphTimeCentric: integrating measurement from " + integrator.first + " was " +
                            (integrationSuccessfully ? "successful" : "failed!"));
    }

    if (paramPtr_->verbose) {
      this->print("GraphTimeCentric: ");
      values_.print("Valued: ");
    }

    if (integrationSuccessfully)
      return StatusGraphConstruction::SUCCESSFUL;
    else
      return StatusGraphConstruction::FAILED;
  }

  double GraphTimeCentric::optimize(data::State &new_state) {
    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();

    solver_->update(*this, values_, keyTimestampMap_, gtsam::FactorIndices(), relatedKeys_);
    gtsam::Values result = solver_->calculateEstimate();
    gtsam::Marginals marginals;

    currentKeyIndexTimestampMap_ = solver_->keyIndexTimestamps();

    new_state.timestamp = fgo::core::TimeStamp(keyTimestampMap_[X(nState_)]);

    try {
      marginals = solver_->getMarginals(result);
      new_state.poseVar = marginals.marginalCovariance(X(nState_));
      new_state.velVar = marginals.marginalCovariance(V(nState_));
      new_state.imuBiasVar = marginals.marginalCovariance(B(nState_));
      
      if (graphBaseParamPtr_->addEstimatedVarianceAfterInit) {
        preIntegratorParams_->biasAccCovariance = new_state.imuBiasVar.block<3, 3>(0, 0);
        preIntegratorParams_->biasOmegaCovariance = new_state.imuBiasVar.block<3, 3>(3, 3);
      }

      if (paramPtr_->addConstDriftFactor) {
        new_state.cbd = result.at<gtsam::Vector2>(C(nState_));
        new_state.cbdVar = marginals.marginalCovariance(C(nState_));
      }
      
      if (paramPtr_->addGPPriorFactor || paramPtr_->addGPInterpolatedFactor) {
        new_state.omega = result.at<gtsam::Vector3>(W(nState_));
        new_state.omegaVar = marginals.marginalCovariance(W(nState_));
      }

      if (graphBaseParamPtr_->publishResiduals) {
        std::vector<gtsam::NonlinearFactor::shared_ptr> factorVec;
        std::for_each(begin(), end(), [&](const gtsam::NonlinearFactor::shared_ptr &factor) -> void {
          factorVec.emplace_back(factor);
        });
        factorBuffer_.update_buffer(factorVec, new_state.timestamp);
        
        if (graphBaseParamPtr_->publishResidualsOnline) {
          resultMarginalBuffer_.update_buffer(std::make_pair(result, marginals), new_state.timestamp);
          // factorBuffer_.cleanBeforeTime(currentKeyIndexTimestampMap_.begin()->second); // Method doesn't exist in CircularDataBuffer
        } else {
          this->calculateResiduals(new_state.timestamp, result, marginals);
        }
      }
    }
    catch (std::exception &ex) {
      appPtr_->getLogger().error("SOLVING with exception: " + std::string(ex.what()));
    }

    solver_->setMarginalizing();
    new_state.state = gtsam::NavState(result.at<gtsam::Pose3>(X(nState_)),
                                      result.at<gtsam::Vector3>(V(nState_)));

    new_state.imuBias = result.at<gtsam::imuBias::ConstantBias>(B(nState_));
    new_state.accMeasured = accBuffer_.get_buffer_from_id(nState_ - 1);

    /*
     * Fetching results for all integrator
     */

    bool fetchingResultsSuccessful = true;
    for (const auto &integrator: integratorMap_) {
      appPtr_->getLogger().info("GraphTimeCentric: starting fetching results for the integrator: " + 
                            integrator.first);

      fetchingResultsSuccessful &= integrator.second->fetchResult(result,
                                                                  marginals,
                                                                  currentKeyIndexTimestampMap_,
                                                                  new_state);

      appPtr_->getLogger().info("GraphTimeCentric: fetching results for the integrator " + 
                            integrator.first + " was " + 
                            (fetchingResultsSuccessful ? "successful" : "failed!"));
    }

    this->resetGraph();
    auto timeOpt = std::chrono::duration_cast<std::chrono::duration<double>>(
        std::chrono::system_clock::now() - start).count();
    
    if (graphBaseParamPtr_->verbose) {
      appPtr_->getLogger().info("Finished graph optimization, fetching results ..., TimeOpt: " + 
                            std::to_string(timeOpt));
    }

    return timeOpt;
  }

}
