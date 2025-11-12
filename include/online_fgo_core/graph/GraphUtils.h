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

#ifndef ONLINE_FGO_CORE_GRAPHUTILS_H
#define ONLINE_FGO_CORE_GRAPHUTILS_H

#pragma once

#include <gtsam/linear/NoiseModel.h>
#include <iostream>
#include <iomanip>
#include "online_fgo_core/data/DataTypesFGO.h"
#include "online_fgo_core/interface/LoggerInterface.h"
#include "online_fgo_core/interface/TimeInterface.h"

namespace fgo::graph {

  /***
   * Query the predicted state at a given time from a time-state buffer
   * @param timeStatePairs Vector of timestamp-state pairs
   * @param timeToQuery Query time in seconds
   * @param logger Optional logger for warnings (nullptr = std::cout)
   * @return Interpolated or nearest state
   */
  inline fgo::data::State
  queryCurrentPredictedState(const std::vector<std::pair<fgo::core::TimeStamp, fgo::data::State>> &timeStatePairs,
                             const double &timeToQuery,
                             fgo::core::LoggerInterface* logger = nullptr) {
    if (timeStatePairs.empty()) {
      if (logger) {
        logger->warn("queryCurrentPredictedState: time/state pairs are empty at time " + 
                     std::to_string(timeToQuery));
      } else {
        std::cout << "WARN: queryCurrentPredictedState: time/state pairs are empty at time " 
                  << std::fixed << timeToQuery << std::endl;
      }
      return {};
    } else if (timeStatePairs.size() == 1) {
      const auto& pair = timeStatePairs.front();
      if (logger) {
        logger->warn("queryCurrentPredictedState: time/state pairs only have one state at " +
                     std::to_string(pair.first.seconds()) + " wished query time: " + 
                     std::to_string(timeToQuery));
      } else {
        std::cout << "WARN: queryCurrentPredictedState: time/state pairs only have one state at "
                  << std::fixed << pair.first.seconds() << " wished query time: " 
                  << timeToQuery << std::endl;
      }
      return pair.second;
    }

    auto itAfter = std::lower_bound(timeStatePairs.begin(), timeStatePairs.end(), timeToQuery,
                                    [](const std::pair<fgo::core::TimeStamp, fgo::data::State> &pair,
                                       double timestamp) -> bool {
                                      // true, ... true, true, false(HERE), false, ... false
                                      return pair.first.seconds() <= timestamp;
                                    });
    if (itAfter == timeStatePairs.begin()) {
      if (logger) {
        logger->warn("queryCurrentPredictedState not found for the time " + std::to_string(timeToQuery));
      } else {
        std::cout << "WARN: queryCurrentPredictedState not found for the time " 
                  << std::fixed << timeToQuery << std::endl;
      }
      return itAfter->second;
    }
    else
      return (itAfter - 1)->second;
  }

  /***
   * Assign noise model based on type and parameters
   * @param modeType Noise model type (Gaussian, Cauchy, Huber, etc.)
   * @param variance Variance vector
   * @param robustParam Robust estimator parameter
   * @param factor Factor name for logging
   * @param logger Optional logger for warnings
   * @return Configured GTSAM noise model
   */
  inline gtsam::SharedNoiseModel assignNoiseModel(fgo::data::NoiseModel modeType,
                                                  const gtsam::Vector &variance,
                                                  double robustParam,
                                                  const std::string &factor = "",
                                                  fgo::core::LoggerInterface* logger = nullptr) {
    gtsam::SharedNoiseModel model = gtsam::noiseModel::Diagonal::Variances(variance);
    switch (modeType) {
      case fgo::data::NoiseModel::GAUSSIAN: {
        return model;
      }
      case fgo::data::NoiseModel::CAUCHY: {
        return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Cauchy::Create(robustParam), model);
      }
      case fgo::data::NoiseModel::HUBER: {
        return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(robustParam), model);
      }
      case fgo::data::NoiseModel::DCS: {
        return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::DCS::Create(robustParam), model);
      }
      case fgo::data::NoiseModel::Tukey: {
        return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Tukey::Create(robustParam), model);
      }
      case fgo::data::NoiseModel::GemanMcClure: {
        return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::GemanMcClure::Create(robustParam),
                                                 model);
      }
      case fgo::data::NoiseModel::Welsch: {
        return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Welsch::Create(robustParam), model);
      }
      default: {
        if (logger) {
          logger->warn("UNKNOWN noise model for factor " + factor);
        } else {
          std::cout << "WARN: UNKNOWN noise model for factor " << factor << std::endl;
        }
        return model;
      }
    }

  }


}
#endif //ONLINE_FGO_CORE_GRAPHUTILS_H
