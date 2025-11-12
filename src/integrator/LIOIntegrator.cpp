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

#include "online_fgo_core/integrator/LIOIntegrator.h"

namespace fgo::integrator {
  void LIOIntegrator::initialize(fgo::core::ApplicationInterface &app, 
                                 fgo::graph::GraphBase &graphPtr, 
                                 const std::string &integratorName,
                                 bool isPrimarySensor) {
    IntegratorBase::initialize(app, graphPtr, integratorName, isPrimarySensor);
    appPtr_->logger().info("--------------------- " + integratorName + 
                          ": start initialization... ---------------------");

    integratorParamPtr_ = std::make_shared<IntegratorOdomParams>(integratorBaseParamPtr_);

    integratorParamPtr_->notIntegrating = 
        appPtr_->parameters().get<bool>("GNSSFGO." + integratorName_ + ".notIntegrating", false);

    integratorParamPtr_->integrateBetweenPose = 
        appPtr_->parameters().get<bool>("GNSSFGO." + integratorName_ + ".integrateBetweenPose", true);
    appPtr_->logger().info("LIOIntegrator integrateBetweenPose: " + 
                          std::to_string(integratorParamPtr_->integrateBetweenPose));

    integratorParamPtr_->integrateGlobalPose = 
        appPtr_->parameters().get<bool>("GNSSFGO." + integratorName_ + ".integrateGlobalPose", false);
    appPtr_->logger().info("LIOIntegrator integrateGlobalPose: " + 
                          std::to_string(integratorParamPtr_->integrateGlobalPose));

    auto odomPoseVarVec = appPtr_->parameters().get<std::vector<double>>(
        "GNSSFGO." + integratorName_ + ".odomPoseVar", 
        std::vector<double>{0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
    integratorParamPtr_->odomPoseVar = gtsam::Vector6(odomPoseVarVec.data());

    auto noiseModelOdomPoseStr = appPtr_->parameters().get<std::string>(
        "GNSSFGO." + integratorName_ + ".noiseModelOdomPose", "gaussian");
    setNoiseModelFromParam(noiseModelOdomPoseStr, integratorParamPtr_->noiseModelOdomPose, 
                          &appPtr_->logger(), "LIO");
    integratorBaseParamPtr_->noiseModelOdomPose = integratorParamPtr_->noiseModelOdomPose;
    appPtr_->logger().info("noiseModelOdomPose: " + noiseModelOdomPoseStr);

    integratorParamPtr_->robustParamOdomPose = 
        appPtr_->parameters().get<double>("GNSSFGO." + integratorName_ + ".robustParamOdomPose", 0.5);
    integratorBaseParamPtr_->robustParamOdomPose = integratorParamPtr_->robustParamOdomPose;

    LIOSAM_ = std::make_shared<sensors::LiDAR::LIOSAM::LIOSAMOdometry>(
        app, 
        integratorParamPtr_,
        integratorName_,
        pubSensorReport_,
        sensorCalibManager_->getTransformationFromBase(sensorName_));
    
    appPtr_->logger().info("--------------------- LIOIntegrator initialized! ---------------------");
  };

  bool LIOIntegrator::addFactors(
      const boost::circular_buffer<std::pair<double, gtsam::Vector3>> &timestampGyroMap,
      const boost::circular_buffer<std::pair<size_t, gtsam::Vector6>> &stateIDAccMap,
      const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap &currentKeyIndexTimestampMap,
      std::vector<std::pair<fgo::core::TimeStamp, fgo::data::State>> &timePredStates,
      gtsam::Values &values,
      fgo::solvers::FixedLagSmoother::KeyTimestampMap &keyTimestampMap,
      gtsam::KeyVector &relatedKeys) {

    std::shared_ptr<fgo::models::GPInterpolator> interpolatorI, interpolatorJ;
    nState_ = currentKeyIndexTimestampMap.end()->first;

    static std::vector<fgo::data::Odom> dataSensorLocalBuffer;
    static uint64_t counter = 0;
    auto dataSensor = LIOSAM_->getOdomAndClean();

    if (dataSensor.empty()) {
      appPtr_->logger().error("LIOSAM: no odom available.");
      return true;
    }

    uint32_t numOdom_ = 0;
    for (const auto &odom: dataSensor) {
      bool hasBetweenPose = checkStatePresentedInCurrentLag(odom.queryOutputPrevious.keyIndexI);

      if (integratorParamPtr_->gpType == fgo::data::GPModelType::WNOJ) {
        interpolatorI = std::make_shared<fgo::models::GPWNOJInterpolator>(
            gtsam::noiseModel::Diagonal::Variances(integratorParamPtr_->QcGPInterpolatorFull), 0, 0,
            integratorParamPtr_->AutoDiffGPInterpolatedFactor, integratorParamPtr_->GPInterpolatedFactorCalcJacobian);
        interpolatorJ = std::make_shared<fgo::models::GPWNOJInterpolator>(
            gtsam::noiseModel::Diagonal::Variances(integratorParamPtr_->QcGPInterpolatorFull), 0, 0,
            integratorParamPtr_->AutoDiffGPInterpolatedFactor, integratorParamPtr_->GPInterpolatedFactorCalcJacobian);
      } else if (integratorParamPtr_->gpType == fgo::data::GPModelType::WNOA) {
        interpolatorI = std::make_shared<fgo::models::GPWNOAInterpolator>(
            gtsam::noiseModel::Diagonal::Variances(integratorParamPtr_->QcGPInterpolatorFull), 0, 0,
            integratorParamPtr_->AutoDiffGPInterpolatedFactor, integratorParamPtr_->GPInterpolatedFactorCalcJacobian);
        interpolatorJ = std::make_shared<fgo::models::GPWNOAInterpolator>(
            gtsam::noiseModel::Diagonal::Variances(integratorParamPtr_->QcGPInterpolatorFull), 0, 0,
            integratorParamPtr_->AutoDiffGPInterpolatedFactor, integratorParamPtr_->GPInterpolatedFactorCalcJacobian);
      } else {
        appPtr_->logger().warn("LIOIntegrator::addFactors NO gpType chosen. Please choose.");
      }

      if (!hasBetweenPose) {
        appPtr_->logger().error("LIOSAM: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Not integrating this odom because previous state with id " +
                               std::to_string(odom.queryOutputPrevious.keyIndexI) + 
                               " is not presented in current lag!");
        std::cout << "Lastlag Keys: " << std::endl;
        for (const auto &k: lastLagKeys_) {
          std::cout << k << " : ";
        }
        std::cout << std::endl;

        LIOSAM_->retractKeyPoseCounter(1);
        LIOSAM_->notifyOptimization();
      }

      if (hasBetweenPose) {
        numOdom_++;
        fgo::data::OdomResult this_result;
        const auto noise_model = graph::assignNoiseModel(integratorBaseParamPtr_->noiseModelOdomPose,
                                                         odom.noise,
                                                         integratorBaseParamPtr_->robustParamOdomPose,
                                                         "odomBetweenFactor");

        this_result.frameIndexCurrent = odom.frameIndexCurrent;
        this_result.frameIndexPrevious = odom.frameIndexPrevious;
        this_result.timestampPrevious = odom.timestampPrevious;
        this_result.timestampCurrent = odom.timestampCurrent;
        this_result.keyIndexII = odom.queryOutputPrevious.keyIndexI;
        this_result.keyIndexIJ = odom.queryOutputPrevious.keyIndexJ;
        this_result.keyIndexJI = odom.queryOutputCurrent.keyIndexI;
        this_result.keyIndexJJ = odom.queryOutputCurrent.keyIndexJ;
        this_result.timestampII = odom.queryOutputPrevious.timestampI;
        this_result.timestampIJ = odom.queryOutputPrevious.timestampJ;
        this_result.timestampJI = odom.queryOutputCurrent.timestampI;
        this_result.timestampJJ = odom.queryOutputCurrent.timestampJ;
        this_result.durationII = odom.queryOutputPrevious.durationI;
        this_result.durationJI = odom.queryOutputCurrent.durationI;
        this_result.keyISynchronized = odom.queryOutputPrevious.keySynchronized;
        this_result.keyJSynchronized = odom.queryOutputCurrent.keySynchronized;
        this_result.posePreviousIMUECEFQueried = odom.queryOutputPrevious.poseIMUECEF;
        this_result.poseCurrentIMUECEFQueried = odom.queryOutputCurrent.poseIMUECEF;

        appPtr_->logger().warn("LIOSAM: THIS ODOM IDs II IJ JI JJ " + 
                              std::to_string(this_result.keyIndexII) + " : " +
                              std::to_string(this_result.keyIndexIJ) + " : " + 
                              std::to_string(this_result.keyIndexJI) + " : " +
                              std::to_string(this_result.keyIndexJJ) +
                              " **************************************");

        if (!odom.queryOutputPrevious.keySynchronized) {
          const double delta_t = odom.queryOutputPrevious.timestampJ - odom.queryOutputPrevious.timestampI;
          if (integratorParamPtr_->gpType == data::GPModelType::WNOJ) {
            this_result.accII = odom.queryOutputPrevious.accI;
            this_result.accIJ = odom.queryOutputPrevious.accJ;
            interpolatorI->recalculate(delta_t, odom.queryOutputPrevious.durationI, odom.queryOutputPrevious.accI,
                                      odom.queryOutputPrevious.accJ);
          } else
            interpolatorI->recalculate(delta_t, odom.queryOutputPrevious.durationI);
        }

        if (!odom.queryOutputCurrent.keySynchronized) {
          const double delta_t = odom.queryOutputCurrent.timestampJ - odom.queryOutputCurrent.timestampI;
          if (integratorParamPtr_->gpType == data::GPModelType::WNOJ) {
            this_result.accJI = odom.queryOutputCurrent.accI;
            this_result.accJJ = odom.queryOutputCurrent.accJ;
            interpolatorJ->recalculate(delta_t, odom.queryOutputCurrent.durationI, odom.queryOutputCurrent.accI,
                                      odom.queryOutputCurrent.accJ);
          } else
            interpolatorJ->recalculate(delta_t, odom.queryOutputCurrent.durationI);
        }

        // NOTICE: LiDAR ODOM is already transformed in IMU CS

        if (integratorParamPtr_->integrateBetweenPose) {

          relatedKeys.emplace_back(X(odom.queryOutputPrevious.keyIndexI));
          relatedKeys.emplace_back(X(odom.queryOutputCurrent.keyIndexJ));

          if (!odom.queryOutputPrevious.keySynchronized && !odom.queryOutputCurrent.keySynchronized) {
            // both are interpolated
            if (!integratorParamPtr_->notIntegrating) {
              appPtr_->logger().info("LIOSAM: Integrating DOUBLE BETWEEN Factor");
              graphPtr_->emplace_shared<fgo::factor::GPInterpolatedDoublePose3BetweenFactor>(
                  X(odom.queryOutputPrevious.keyIndexI), V(odom.queryOutputPrevious.keyIndexI),
                  W(odom.queryOutputPrevious.keyIndexI),
                  X(odom.queryOutputPrevious.keyIndexJ), V(odom.queryOutputPrevious.keyIndexJ),
                  W(odom.queryOutputPrevious.keyIndexJ),
                  X(odom.queryOutputCurrent.keyIndexI), V(odom.queryOutputCurrent.keyIndexI),
                  W(odom.queryOutputCurrent.keyIndexI),
                  X(odom.queryOutputCurrent.keyIndexJ), V(odom.queryOutputCurrent.keyIndexJ),
                  W(odom.queryOutputCurrent.keyIndexJ),
                  odom.poseRelativeECEF, interpolatorI, interpolatorJ, noise_model);
            }
            odomResults_.emplace_back(this_result);
          } else if (!odom.queryOutputPrevious.keySynchronized && odom.queryOutputCurrent.keySynchronized) {
            if (!integratorParamPtr_->notIntegrating) {
              appPtr_->logger().info("LIOSAM: Integrating SINGLE BETWEEN Factor by querying the PREVIOUS state.");
              graphPtr_->emplace_shared<fgo::factor::GPInterpolatedSinglePose3BetweenFactor>(
                  X(odom.queryOutputPrevious.keyIndexI), V(odom.queryOutputPrevious.keyIndexI),
                  W(odom.queryOutputPrevious.keyIndexI),
                  X(odom.queryOutputPrevious.keyIndexJ), V(odom.queryOutputPrevious.keyIndexJ),
                  W(odom.queryOutputPrevious.keyIndexJ),
                  X(odom.queryOutputCurrent.keyIndexI),
                  odom.poseRelativeECEF, false,
                  interpolatorI, noise_model);
            }
            odomResults_.emplace_back(this_result);
          } else if (odom.queryOutputPrevious.keySynchronized && !odom.queryOutputCurrent.keySynchronized) {
            if (!integratorParamPtr_->notIntegrating) {
              appPtr_->logger().info("LIOSAM: Integrating SINGLE BETWEEN Factor by querying the CURRENT state.");
              graphPtr_->emplace_shared<fgo::factor::GPInterpolatedSinglePose3BetweenFactor>(
                  X(odom.queryOutputCurrent.keyIndexI), V(odom.queryOutputCurrent.keyIndexI),
                  W(odom.queryOutputCurrent.keyIndexI),
                  X(odom.queryOutputCurrent.keyIndexJ), V(odom.queryOutputCurrent.keyIndexJ),
                  W(odom.queryOutputCurrent.keyIndexJ),
                  X(odom.queryOutputPrevious.keyIndexI),
                  odom.poseRelativeECEF, true,
                  interpolatorJ, noise_model);
            }
            odomResults_.emplace_back(this_result);

          } else if (odom.queryOutputPrevious.keySynchronized && odom.queryOutputCurrent.keySynchronized) {
            if (!integratorParamPtr_->notIntegrating) {
              appPtr_->logger().info("LIOSAM: Integrating BETWEEN Factor.");
              if (integratorParamPtr_->integrateBetweenPose) {
                auto betweenFactor = boost::make_shared<fgo::factor::BetweenFactor<gtsam::Pose3>>(
                    X(odom.queryOutputPrevious.keyIndexI),
                    X(odom.queryOutputCurrent.keyIndexI),
                    odom.poseRelativeECEF, noise_model);
                graphPtr_->push_back(betweenFactor);
              }
            }
            odomResults_.emplace_back(this_result);
          } else {
            appPtr_->logger().warn("LIOSAM: Cant integrate this odom!");
          }
        }
      }

      if (integratorParamPtr_->integrateGlobalPose) {

        bool noGlobalPose = checkStatePresentedInCurrentLag(odom.queryOutputCurrent.keyIndexI);

        if (!noGlobalPose) {
          appPtr_->logger().error("LIOSAM: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Not integrating this odom as GLOBAL POSE because previous state with id " +
                                 std::to_string(odom.queryOutputCurrent.keyIndexI) + 
                                 " is not presented in current lag!");
          continue;
        }

        if (odom.queryOutputCurrent.keySynchronized) {
          this->addNavPoseFactor(X(odom.queryOutputCurrent.keyIndexI),
                                odom.poseToECEF, integratorParamPtr_->odomPoseVar);
        } else {
          this->addGPinteporatedNavPoseFactor(X(odom.queryOutputCurrent.keyIndexI),
                                             V(odom.queryOutputCurrent.keyIndexI),
                                             W(odom.queryOutputCurrent.keyIndexI),
                                             X(odom.queryOutputCurrent.keyIndexJ),
                                             V(odom.queryOutputCurrent.keyIndexJ),
                                             W(odom.queryOutputCurrent.keyIndexJ),
                                             odom.poseToECEF, integratorParamPtr_->odomPoseVar, interpolatorJ);
        }
      }
      counter++;
    }
    return true;
  }


  std::map<uint64_t, double> LIOIntegrator::factorizeAsPrimarySensor() {

    std::map<uint64_t, double> keyIndexTimestampsMap;
    auto dataSensor = LIOSAM_->getOdomAndClean();

    if (!graphPtr_->isGraphInitialized()) {
      appPtr_->logger().error("PrimarySensor on " + integratorName_ + " graph not initialized!");
      return {};
    }

    for (const auto &odom: dataSensor) {
      nState_++;
      const auto &current_timestamp = odom.timestampCurrent;
      keyIndexTimestampsMap.insert(std::make_pair(nState_, current_timestamp));

      if (integratorParamPtr_->integrateBetweenPose) {
        const auto noise_model = graph::assignNoiseModel(integratorBaseParamPtr_->noiseModelOdomPose,
                                                         odom.noise,
                                                         integratorBaseParamPtr_->robustParamOdomPose,
                                                         "odomBetweenFactor");

        auto betweenFactor = boost::make_shared<fgo::factor::BetweenFactor<gtsam::Pose3>>(X(nState_ - 1),
                                                                                          X(nState_),
                                                                                          odom.poseRelativeECEF,
                                                                                          noise_model);
        graphPtr_->push_back(betweenFactor);
      }

      if (integratorParamPtr_->integrateGlobalPose) {
        this->addNavPoseFactor(X(nState_),
                              odom.poseToECEF, integratorParamPtr_->odomPoseVar);
      }
    }
    return keyIndexTimestampsMap;
  }

  bool LIOIntegrator::fetchResult(const gtsam::Values &result, const gtsam::Marginals &marginals,
                                  const solvers::FixedLagSmoother::KeyIndexTimestampMap &keyIndexTimestampMap,
                                  data::State &optState) {
    static std::shared_ptr<fgo::models::GPInterpolator> interpolatorI, interpolatorJ;
    static bool interpolatorInitialized = false;
    if(!interpolatorInitialized) {
      if (integratorParamPtr_->gpType == fgo::data::GPModelType::WNOJ) {
        interpolatorI = std::make_shared<fgo::models::GPWNOJInterpolator>(
            gtsam::noiseModel::Diagonal::Variances(integratorParamPtr_->QcGPInterpolatorFull), 0, 0,
            integratorParamPtr_->AutoDiffGPInterpolatedFactor, integratorParamPtr_->GPInterpolatedFactorCalcJacobian);
        interpolatorJ = std::make_shared<fgo::models::GPWNOJInterpolator>(
            gtsam::noiseModel::Diagonal::Variances(integratorParamPtr_->QcGPInterpolatorFull), 0, 0,
            integratorParamPtr_->AutoDiffGPInterpolatedFactor, integratorParamPtr_->GPInterpolatedFactorCalcJacobian);
      } else if (integratorParamPtr_->gpType == fgo::data::GPModelType::WNOA) {
        interpolatorI = std::make_shared<fgo::models::GPWNOAInterpolator>(
            gtsam::noiseModel::Diagonal::Variances(integratorParamPtr_->QcGPInterpolatorFull), 0, 0,
            integratorParamPtr_->AutoDiffGPInterpolatedFactor, integratorParamPtr_->GPInterpolatedFactorCalcJacobian);
        interpolatorJ = std::make_shared<fgo::models::GPWNOAInterpolator>(
            gtsam::noiseModel::Diagonal::Variances(integratorParamPtr_->QcGPInterpolatorFull), 0, 0,
            integratorParamPtr_->AutoDiffGPInterpolatedFactor, integratorParamPtr_->GPInterpolatedFactorCalcJacobian);
      } else {
        appPtr_->logger().warn("LIOIntegrator::addFactors NO gpType chosen. Please choose.");
      }
      interpolatorInitialized = true;
    }

    for (const auto &odom: odomResults_) {
      gtsam::Pose3 poseI, poseJ;
      if (odom.keyJSynchronized && odom.keyISynchronized) {
        if (result.exists(X(odom.keyIndexII))) {
          poseI = result.at<gtsam::Pose3>(X(odom.keyIndexII));
        } else {
          appPtr_->logger().warn("Case 1: Continue because no keyindexII in results, should be marginalized out!");
          poseI = odom.posePreviousIMUECEFQueried;
        }
        if (result.exists(X(odom.keyIndexJI))) {
          poseJ = result.at<gtsam::Pose3>(X(odom.keyIndexJI));
        } else {
          appPtr_->logger().warn("Case 1: Continue because no keyindexJI in results, should be marginalized out!");
          poseJ = odom.poseCurrentIMUECEFQueried;
        }

      } else if (odom.keyISynchronized && !odom.keyJSynchronized) {
        if (result.exists(X(odom.keyIndexII))) {
          poseI = result.at<gtsam::Pose3>(X(odom.keyIndexII));
        } else {
          appPtr_->logger().warn("Case 2: Continue because no keyindexII in results, should be marginalized out!");
          poseI = odom.posePreviousIMUECEFQueried;
        }
        interpolatorJ->recalculate(odom.timestampJJ - odom.timestampJI, odom.durationJI);

        if (result.exists(X(odom.keyIndexJI)) && result.exists(X(odom.keyIndexJJ))) {
          poseJ = interpolatorJ->interpolatePose(result.at<gtsam::Pose3>(X(odom.keyIndexJI)),
                                                 result.at<gtsam::Vector3>(V(odom.keyIndexJI)),
                                                 result.at<gtsam::Vector3>(W(odom.keyIndexJI)),
                                                 result.at<gtsam::Pose3>(X(odom.keyIndexJJ)),
                                                 result.at<gtsam::Vector3>(V(odom.keyIndexJJ)),
                                                 result.at<gtsam::Vector3>(W(odom.keyIndexJJ)));
        } else {
          appPtr_->logger().warn("Case 2: Continue because no keyIndexJI or keyIndexJJ in results, should be marginalized out!");
          poseJ = odom.poseCurrentIMUECEFQueried;
        }
      } else if (!odom.keyISynchronized && odom.keyJSynchronized) {
        if (result.exists(X(odom.keyIndexJI))) {
          poseJ = result.at<gtsam::Pose3>(X(odom.keyIndexJI));
        } else {
          appPtr_->logger().warn("Case 3: Continue because no keyindexJI in results, should be marginalized out!");
          poseJ = odom.poseCurrentIMUECEFQueried;
        }

        interpolatorI->recalculate(odom.timestampIJ - odom.timestampII, odom.durationII);

        if (result.exists(X(odom.keyIndexII)) && result.exists(X(odom.keyIndexIJ))) {
          poseI = interpolatorI->interpolatePose(result.at<gtsam::Pose3>(X(odom.keyIndexII)),
                                                 result.at<gtsam::Vector3>(V(odom.keyIndexII)),
                                                 result.at<gtsam::Vector3>(W(odom.keyIndexII)),
                                                 result.at<gtsam::Pose3>(X(odom.keyIndexIJ)),
                                                 result.at<gtsam::Vector3>(V(odom.keyIndexIJ)),
                                                 result.at<gtsam::Vector3>(W(odom.keyIndexIJ)));

        } else {
          appPtr_->logger().warn("Case 3: Continue because no keyIndexII or keyIndexIJ in results, should be marginalized out!");
          poseI = odom.posePreviousIMUECEFQueried;
        }

      } else if (!odom.keyISynchronized && !odom.keyJSynchronized) {
        interpolatorI->recalculate(odom.timestampIJ - odom.timestampII, odom.durationII);
        interpolatorJ->recalculate(odom.timestampJJ - odom.timestampJI, odom.durationJI);

        if (result.exists(X(odom.keyIndexII)) && result.exists(X(odom.keyIndexIJ))) {
          poseI = interpolatorI->interpolatePose(result.at<gtsam::Pose3>(X(odom.keyIndexII)),
                                                 result.at<gtsam::Vector3>(V(odom.keyIndexII)),
                                                 result.at<gtsam::Vector3>(W(odom.keyIndexII)),
                                                 result.at<gtsam::Pose3>(X(odom.keyIndexIJ)),
                                                 result.at<gtsam::Vector3>(V(odom.keyIndexIJ)),
                                                 result.at<gtsam::Vector3>(W(odom.keyIndexIJ)));

        } else {
          appPtr_->logger().warn("Case 4: Continue because no keyIndexII or keyIndexIJ in results, should be marginalized out!");
          poseI = odom.posePreviousIMUECEFQueried;
        }

        if (result.exists(X(odom.keyIndexJI)) && result.exists(X(odom.keyIndexJJ))) {
          poseJ = interpolatorJ->interpolatePose(result.at<gtsam::Pose3>(X(odom.keyIndexJI)),
                                                 result.at<gtsam::Vector3>(V(odom.keyIndexJI)),
                                                 result.at<gtsam::Vector3>(W(odom.keyIndexJI)),
                                                 result.at<gtsam::Pose3>(X(odom.keyIndexJJ)),
                                                 result.at<gtsam::Vector3>(V(odom.keyIndexJJ)),
                                                 result.at<gtsam::Vector3>(W(odom.keyIndexJJ)));

        } else {
          appPtr_->logger().warn("Case 4: Continue because no keyIndexJI or keyIndexJJ in results, should be marginalized out!");
          poseJ = odom.poseCurrentIMUECEFQueried;
        }

      } else {
        appPtr_->logger().warn("Can't update key cloud pose for odom with scan index " + 
                              std::to_string(odom.frameIndexPrevious) + " and " +
                              std::to_string(odom.frameIndexCurrent));
        continue;
      }

      LIOSAM_->updateCloudKeyPose(odom.frameIndexPrevious, poseI, odom.frameIndexCurrent, poseJ);
    }
    odomResults_.clear();

    for (const auto &keyIndexTs: keyIndexTimestampMap) {
      fgo::data::QueryStateInput input;
      input.pose = result.at<gtsam::Pose3>(X(keyIndexTs.first));
      input.vel = result.at<gtsam::Vector3>(V(keyIndexTs.first));
      if (integratorParamPtr_->addGPPriorFactor || integratorParamPtr_->addGPInterpolatedFactor) {
        input.omega = result.at<gtsam::Vector3>(W(keyIndexTs.first));
        input.acc = optState.accMeasured;
      }
      LIOSAM_->updateOptPose(keyIndexTs.first,
                            fgo::core::TimeStamp::fromSeconds(keyIndexTs.second), 
                            input);
    }
    LIOSAM_->updateKeyIndexTimestampMap(keyIndexTimestampMap);
    lastKeyIndexTimestampMap_ = keyIndexTimestampMap;
    lastLagKeys_.clear();
    for (const auto &m: keyIndexTimestampMap)
      lastLagKeys_.emplace_back(m.first);
    
    return true;
  }
}
