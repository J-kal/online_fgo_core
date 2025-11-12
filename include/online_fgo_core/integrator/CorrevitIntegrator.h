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

#ifndef ONLINE_FGO_CORE_CORREVITINTEGRATOR_H
#define ONLINE_FGO_CORE_CORREVITINTEGRATOR_H

#include "online_fgo_core/integrator/IntegratorBase.h"
#include "online_fgo_core/factor/odometry/NavVelocityFactor.h"
#include "online_fgo_core/factor/odometry/GPInterpolatedNavVelocityFactor.h"

namespace fgo::integrator
{
    struct CorrevitVelAngle
    {
        double timestamp{};
        double angle{};
        double vel{};
        double vel_x{};
        double vel_y{};
        double delay = 0.;
    };

    struct Correvit
    {
        double timestamp{};
        double angle_correvit{};
        double vel_x_correvit{};
        double vel_correvit{};
        double vel_y_correvit{};
        double delay = 0.;
    };

    struct CorrevitPitchRoll
    {
        double timestamp{};
        double pitch{};
        double radius{};
        double roll{};
        double delay = 0.;
    };

    class CorrevitIntegrator : public IntegratorBase
    {
    protected:
        fgo::data::CircularDataBuffer<CorrevitVelAngle> bufferCorrevitVelAngle_;
        fgo::data::CircularDataBuffer<Correvit> bufferCorrevit_;
        fgo::data::CircularDataBuffer<CorrevitPitchRoll> bufferCorrevitPitchRoll_;

        IntegratorCorrevitParamsPtr paramPtr_;
        std::shared_ptr<fgo::models::GPInterpolator> interpolator_;

        std::atomic_bool zeroVelocity_ = false;

    public:
        explicit CorrevitIntegrator() = default;
        ~CorrevitIntegrator() override = default;

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
          fgo::solvers::FixedLagSmoother::KeyTimestampMap &keyTimestampMap,
          gtsam::KeyVector& relatedKeys
        ) override;

        bool fetchResult(
            const gtsam::Values& result,
            const gtsam::Marginals& marginals,
            const fgo::solvers::FixedLagSmoother::KeyIndexTimestampMap& keyIndexTimestampMap,
            fgo::data::State& optState
        ) override;

        void dropMeasurementBefore(double timestamp) override{
          bufferCorrevitVelAngle_.cleanBeforeTime(timestamp);
          bufferCorrevit_.cleanBeforeTime(timestamp);
          bufferCorrevitPitchRoll_.cleanBeforeTime(timestamp);
        }

        bool checkZeroVelocity() override{
          return zeroVelocity_;
        }

        bool checkHasMeasurements() override
        {
          return bufferCorrevit_.size() != 0;
        }

        void cleanBuffers() override
        {
          bufferCorrevit_.clean();
        }

        // Public methods for ROS adapter to push data
        void addCorrevitVelAngle(const CorrevitVelAngle& data) {
            bufferCorrevitVelAngle_.update_buffer(data, fgo::core::TimeStamp::fromSeconds(data.timestamp));
        }

        void addCorrevit(const Correvit& data) {
            bufferCorrevit_.update_buffer(data, fgo::core::TimeStamp::fromSeconds(data.timestamp));
            
            // Zero velocity detection
            static double sumVelocity = 0;
            static uint calcZeroVelocityCounter = 1;
            sumVelocity += data.vel_correvit;
            if(calcZeroVelocityCounter > 6) {
                const auto avgVelocity = abs(sumVelocity) / calcZeroVelocityCounter;
                if(avgVelocity < paramPtr_->zeroVelocityThreshold) {
                    appPtr_->logger().warn(integratorName_ + " reported near zero velocity: " + 
                                          std::to_string(data.angle_correvit));
                    bufferCorrevit_.clean();
                    zeroVelocity_ = true;
                } else
                    zeroVelocity_ = false;
                calcZeroVelocityCounter = 0;
                sumVelocity = 0.;
            }
            calcZeroVelocityCounter++;
        }

        void addCorrevitPitchRoll(const CorrevitPitchRoll& data) {
            bufferCorrevitPitchRoll_.update_buffer(data, fgo::core::TimeStamp::fromSeconds(data.timestamp));
        }
    };
}
#endif //ONLINE_FGO_CORE_CORREVITINTEGRATOR_H
