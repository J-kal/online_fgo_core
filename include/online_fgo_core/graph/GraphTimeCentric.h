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

#ifndef ONLINE_FGO_CORE_GRAPHTIMECENTRIC_H
#define ONLINE_FGO_CORE_GRAPHTIMECENTRIC_H

#pragma once

#include <any>
#include "online_fgo_core/graph/GraphBase.h"

namespace fgo::graph
{
    // Forward declaration for integrator (will be migrated later)
    namespace integrator {
        class LIOIntegrator;
    }

    // Message type for sensor processing report
    struct SensorProcessingReport {
        double timestamp;
        std::string sensor_name;
        int measurements_processed;
        double processing_time_ms;
    };

    class GraphTimeCentric : public GraphBase
    {
        GraphTimeCentricParamPtr paramPtr_;
        std::shared_ptr<fgo::core::PublisherInterface<SensorProcessingReport>> pubIMUFactorReport_;

    public:
        typedef std::shared_ptr<GraphTimeCentric> Ptr;

        explicit GraphTimeCentric(fgo::core::ApplicationInterface& app);

        ~GraphTimeCentric() override
        {
            if(pubResidualsThread_ && pubResidualsThread_->joinable()) {
                shouldPublishResiduals_ = false;
                pubResidualsThread_->join();
            }
        }

        StatusGraphConstruction constructFactorGraphOnIMU(
            std::vector<fgo::data::IMUMeasurement>& dataIMU
        ) override;

        StatusGraphConstruction constructFactorGraphOnTime(
            const std::vector<double>& stateTimestamps,
            std::vector<fgo::data::IMUMeasurement> &dataIMU
        ) override;

        double optimize(fgo::data::State& new_state) override;
    };
}

#endif //ONLINE_FGO_CORE_GRAPHTIMECENTRIC_H
