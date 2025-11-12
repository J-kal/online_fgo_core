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

#include "online_fgo_core/integrator/CorrevitIntegrator.h"

namespace fgo::integrator
{
    void CorrevitIntegrator::initialize(fgo::core::ApplicationInterface &app, 
                                        graph::GraphBase &graphPtr, 
                                        const std::string& integratorName, 
                                        bool isPrimarySensor) {
      IntegratorBase::initialize(app, graphPtr, integratorName, isPrimarySensor);
      appPtr_->logger().info("--------------------- " + integratorName + 
                            ": start initialization... ---------------------");

      paramPtr_ = std::make_shared<IntegratorCorrevitParams>(integratorBaseParamPtr_);
      
      /**
       *  Parameters
       */
      paramPtr_->integrateVelocity = 
          appPtr_->parameters().get<bool>("GNSSFGO." + integratorName_ + ".integrateVelocity", true);

      paramPtr_->fixedVelVar = 
          appPtr_->parameters().get<double>("GNSSFGO." + integratorName_ + ".fixedVelVar", 1.0);

      paramPtr_->velVarScale = 
          appPtr_->parameters().get<double>("GNSSFGO." + integratorName_ + ".velVarScale", 1.0);

      paramPtr_->robustParamVelocity = 
          appPtr_->parameters().get<double>("GNSSFGO." + integratorName_ + ".roustParamVel", 1.0);
      integratorBaseParamPtr_->robustParamVelocity = paramPtr_->robustParamVelocity;

      auto velocityFrameStr = appPtr_->parameters().get<std::string>(
          "GNSSFGO." + integratorName_ + ".velocityFrame", "body");
      setSensorFrameFromParam(velocityFrameStr, paramPtr_->velocityFrame, 
                             &appPtr_->logger(), "Correvit");
      appPtr_->logger().info("velocityFrame: " + velocityFrameStr);
      integratorBaseParamPtr_->velocityFrame = paramPtr_->velocityFrame;

      auto velocityTypeStr = appPtr_->parameters().get<std::string>(
          "GNSSFGO." + integratorName_ + ".velocityType", "3d");
      setVelocityType(velocityTypeStr, paramPtr_->velocityType, 
                     &appPtr_->logger(), "Correvit");
      appPtr_->logger().info("velocityType: " + velocityTypeStr);
      integratorBaseParamPtr_->velocityType = paramPtr_->velocityType;

      auto noiseModelVelocityStr = appPtr_->parameters().get<std::string>(
          "GNSSFGO." + integratorName_ + ".noiseModelVelocity", "gaussian");
      setNoiseModelFromParam(noiseModelVelocityStr, paramPtr_->noiseModelVelocity, 
                            &appPtr_->logger(), "GNSSLC Velocity");
      integratorBaseParamPtr_->noiseModelVelocity = paramPtr_->noiseModelVelocity;
      appPtr_->logger().info("noiseModelVelocity: " + noiseModelVelocityStr);

      appPtr_->logger().info("AutoDiff: " + std::to_string(paramPtr_->AutoDiffNormalFactor));

      paramPtr_->nearZeroVelocityThreshold = 
          appPtr_->parameters().get<double>("GNSSFGO." + integratorName_ + ".nearZeroVelocityThreshold", 0.0);

      paramPtr_->StateMeasSyncUpperBound = 
          appPtr_->parameters().get<double>("GNSSFGO." + integratorName_ + ".StateMeasSyncUpperBound", 0.02);
      paramPtr_->StateMeasSyncLowerBound = 
          appPtr_->parameters().get<double>("GNSSFGO." + integratorName_ + ".StateMeasSyncLowerBound", -0.02);

      auto velVarVector = appPtr_->parameters().get<std::vector<double>>(
          "GNSSFGO." + integratorName_ + ".velVarVector", 
          std::vector<double>{0.1, 0.1, 0.1});
      paramPtr_->velVar = gtsam::Vector3(velVarVector.data());

      paramPtr_->factorizeDelay = 
          appPtr_->parameters().get<double>("GNSSFGO." + integratorName_ + ".factorizeDelay", 5.0);
      appPtr_->logger().info("factorizeDelay: " + std::to_string(paramPtr_->factorizeDelay));

      paramPtr_->enablePreIntegration = 
          appPtr_->parameters().get<bool>("GNSSFGO." + integratorName_ + ".enablePreIntegration", false);

      paramPtr_->zeroVelocityThreshold = 
          appPtr_->parameters().get<double>("GNSSFGO." + integratorName_ + ".zeroVelocityThreshold", 0.1);
      appPtr_->logger().info("zeroVelocityThreshold: " + std::to_string(paramPtr_->zeroVelocityThreshold));

      bufferCorrevitVelAngle_.resize_buffer(1000);
      bufferCorrevit_.resize_buffer(1000);
      bufferCorrevitPitchRoll_.resize_buffer(1000);

      appPtr_->logger().info("--------------------- " + integratorName_ + 
                            " initialized! ---------------------");
    }

    bool CorrevitIntegrator::addFactors(
        const boost::circular_buffer<std::pair<double, gtsam::Vector3>> &timestampGyroMap,
        const boost::circular_buffer<std::pair<size_t, gtsam::Vector6>>& stateIDAccMap,
        const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap &currentKeyIndexTimestampMap,
        std::vector<std::pair<fgo::core::TimeStamp, fgo::data::State>>& timePredStates,
        gtsam::Values &values,
        solvers::FixedLagSmoother::KeyTimestampMap &keyTimestampMap,
        gtsam::KeyVector& relatedKeys) {

      static auto firstCallTime = appPtr_->time().now();
      static auto fixedVelVar = paramPtr_->velVar * paramPtr_->velVarScale;
      static auto baseToSensorTrans = sensorCalibManager_->getTransformationFromBase(sensorName_);

      if(!paramPtr_->integrateVelocity) {
        return true;
      }

      if((appPtr_->time().now() - firstCallTime).seconds() < paramPtr_->factorizeDelay) {
        appPtr_->logger().warn("Correvit: delaying");
        bufferCorrevit_.clean();
        return true;
      }

      static gtsam::Key pose_key_j, vel_key_j, bias_key_j, omega_key_j,
             pose_key_i, vel_key_i,  bias_key_i, omega_key_i,
             pose_key_sync, vel_key_sync,  bias_key_sync;
      static uint64_t last_key_index = 0;
      static boost::circular_buffer<CorrevitVelAngle> restCorrevitVelAngle(1000);
      static boost::circular_buffer<Correvit> restCorrevit(1000);
      static boost::circular_buffer<CorrevitPitchRoll> restCorrevitPitchRoll(1000);

      auto dataCorrevit = bufferCorrevit_.get_all_buffer_and_clean();

      if(!restCorrevit.empty()) {
        dataCorrevit.insert(dataCorrevit.begin(), restCorrevit.begin(), restCorrevit.end());
        restCorrevit.clear();
      }

      appPtr_->logger().warn("Correvit: integrating with " + std::to_string(dataCorrevit.size()) + 
                            " correvit data");

      auto dataCorrevitIter = dataCorrevit.begin();
      while(dataCorrevitIter != dataCorrevit.end()) {
        const auto corrected_time = dataCorrevitIter->timestamp - 0.0008;
        const auto vel = (gtsam::Vector3() << dataCorrevitIter->vel_x_correvit, 
                         -dataCorrevitIter->vel_y_correvit, 0).finished();

        if(dataCorrevitIter->vel_x_correvit < paramPtr_->nearZeroVelocityThreshold) {
          appPtr_->logger().info("[Correvit] near zero velocity with vx: " + 
                                std::to_string(dataCorrevitIter->vel_x_correvit) + 
                                " not integrating...");
          dataCorrevitIter++;
          continue;
        }

        auto syncResult = stateMeasSynchronize(corrected_time, currentKeyIndexTimestampMap, paramPtr_);
        if (syncResult.status == StateMeasSyncStatus::DROPPED || 
            syncResult.status == StateMeasSyncStatus::INTERPOLATED) {
          dataCorrevitIter++;
          continue;
        }

        const auto currentPredState = timePredStates.back().second;

        if(paramPtr_->verbose) {
          std::cout << "Correvit Measured X: " << dataCorrevitIter->vel_x_correvit << std::endl;
          std::cout << "Correvit Measured Y: " << dataCorrevitIter->vel_y_correvit << std::endl;
          const auto test = currentPredState.state.attitude().unrotate(currentPredState.state.velocity());
          std::cout << "current vel_b: " << test << std::endl;
        }

        pose_key_i = X(syncResult.keyIndexI);
        vel_key_i = V(syncResult.keyIndexI);
        bias_key_i = B(syncResult.keyIndexI);
        omega_key_i = W(syncResult.keyIndexI);

        pose_key_j = X(syncResult.keyIndexJ);
        vel_key_j = V(syncResult.keyIndexJ);
        bias_key_j = B(syncResult.keyIndexJ);
        omega_key_j = W(syncResult.keyIndexJ);

        if (!syncResult.stateJExist()) {
          appPtr_->logger().error("GP Correvit: NO state J found!!!");
        }

        double timeSync;
        if (syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_I || 
            syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_J) {

          if (syncResult.status == StateMeasSyncStatus::SYNCHRONIZED_I) {
            pose_key_sync = pose_key_i;
            vel_key_sync = vel_key_i;
            bias_key_sync = bias_key_i;
            timeSync = syncResult.timestampI;
          } else {
            pose_key_sync = pose_key_j;
            vel_key_sync = vel_key_j;
            bias_key_sync = bias_key_j;
            timeSync = syncResult.timestampJ;
          }

          const auto [foundGyro, this_gyro] = findOmegaToMeasurement(corrected_time, timestampGyroMap);
          const auto this_gyro_unbiased = currentPredState.imuBias.correctGyroscope(this_gyro);

          if(last_key_index == gtsam::symbolIndex(pose_key_sync)) {
            dataCorrevitIter++;
            continue;
          }

          this->addNavVelocityFactor(pose_key_sync, vel_key_sync, vel, fixedVelVar, this_gyro_unbiased,
                                     baseToSensorTrans.translation(), paramPtr_->velocityType);
        } else if(syncResult.status == StateMeasSyncStatus::CACHED) {
          restCorrevit.push_back(*dataCorrevitIter);
        }

        last_key_index = gtsam::symbolIndex(pose_key_sync);
        dataCorrevitIter++;
      }

      return true;
    }

    bool CorrevitIntegrator::fetchResult(const gtsam::Values &result, 
                                         const gtsam::Marginals &marginals,
                                         const solvers::FixedLagSmoother::KeyIndexTimestampMap &keyIndexTimestampMap,
                                         data::State &optState) {
      return true;
    }
}
