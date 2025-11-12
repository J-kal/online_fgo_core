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

#ifndef ONLINE_FGO_CORE_LIOINTEGRATOR_H
#define ONLINE_FGO_CORE_LIOINTEGRATOR_H

#include "online_fgo_core/integrator/IntegratorBase.h"
#include "online_fgo_core/factor/odometry/BetweenFactor.h"
#include "online_fgo_core/factor/odometry/GPInterpolatedDoublePose3BetweenFactor.h"
#include "online_fgo_core/factor/odometry/GPInterpolatedSinglePose3BetweenFactor.h"
#include "online_fgo_core/sensor/lidar/LIOSAM.h"
#include "online_fgo_core/sensor/lidar/LIOSAMUtils.h"

namespace fgo::integrator
{
    using namespace sensors::LiDAR::LIOSAM;

    class LIOIntegrator : public IntegratorBase
    {
        IntegratorOdomParamsPtr integratorParamPtr_;
        std::vector<fgo::data::OdomResult> odomResults_;
        std::shared_ptr<sensors::LiDAR::LIOSAM::LIOSAMOdometry> LIOSAM_;

    public:
        explicit LIOIntegrator() = default;
        ~LIOIntegrator() override = default;

        void initialize(fgo::core::ApplicationInterface& app, 
                       fgo::graph::GraphBase& graphPtr, 
                       const std::string& integratorName, 
                       bool isPrimarySensor = false) override;

        bool addFactors(
          const boost::circular_buffer<std::pair<double, gtsam::Vector3>>& timestampGyroMap,
          const boost::circular_buffer<std::pair<size_t, gtsam::Vector6>>& stateIDAccMap,
          const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap &currentKeyIndexTimestampMap,
          std::vector<std::pair<fgo::core::TimeStamp, fgo::data::State>>& timePredStates,
          gtsam::Values& values,
          fgo::solvers::FixedLagSmoother::KeyTimestampMap& keyTimestampMap,
          gtsam::KeyVector& relatedKeys) override;

        std::map<uint64_t, double> factorizeAsPrimarySensor() override;

        bool fetchResult(
            const gtsam::Values& result,
            const gtsam::Marginals& marginals,
            const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap& keyIndexTimestampMap,
            fgo::data::State& optState
        ) override;

        bool checkHasMeasurements() override
        {
          return !LIOSAM_->hasOdom();
        }

        void cleanBuffers() override
        {
          LIOSAM_->cleanBuffer();
        }

        void notifyOptimization(double noOptimizationDuration) override
        {
          noOptimizationDuration_ = noOptimizationDuration;
          LIOSAM_->notifyOptimization();
        }
    };
}
#endif //ONLINE_FGO_CORE_LIOINTEGRATOR_H
