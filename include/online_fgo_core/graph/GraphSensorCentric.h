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

#ifndef ONLINE_FGO_CORE_GRAPHSENSORCENTRIC_H
#define ONLINE_FGO_CORE_GRAPHSENSORCENTRIC_H

#pragma once

#include "online_fgo_core/graph/GraphTimeCentric.h"
#include "online_fgo_core/graph/param/GraphSensorCentricParams.h"

namespace fgo::graph
{
    /**
     * @brief Sensor-centric graph implementation
     * 
     * This graph variant organizes optimization around sensor measurements
     * rather than time intervals. Useful for sensor-driven architectures
     * where measurements arrive asynchronously.
     */
    class GraphSensorCentric : public GraphTimeCentric
    {
        GraphSensorCentricParamPtr paramPtr_;

    public:
        typedef std::shared_ptr<GraphSensorCentric> Ptr;

        /**
         * @brief Construct a new Graph Sensor Centric object
         * 
         * @param app Application interface providing framework-agnostic services
         */
        explicit GraphSensorCentric(fgo::core::ApplicationInterface& app);

        /**
         * @brief Destructor ensuring proper cleanup
         */
        ~GraphSensorCentric() override
        {
            if(pubResidualsThread_ && pubResidualsThread_->joinable()) {
                shouldPublishResiduals_ = false;
                pubResidualsThread_->join();
            }
        }

        /**
         * @brief Construct factor graph using IMU as timing reference
         * 
         * @param dataIMU IMU measurements to process
         * @return StatusGraphConstruction Status of graph construction
         */
        StatusGraphConstruction constructFactorGraphOnIMU(
                std::vector<fgo::data::IMUMeasurement>& dataIMU
        ) override;

        /**
         * @brief Construct factor graph using fixed time intervals
         * 
         * @param stateTimestamps Target timestamps for states
         * @param dataIMU IMU measurements for interpolation
         * @return StatusGraphConstruction Status of graph construction
         */
        StatusGraphConstruction constructFactorGraphOnTime(
                const std::vector<double>& stateTimestamps,
                std::vector<fgo::data::IMUMeasurement> &dataIMU
        ) override;

    };
}

#endif //ONLINE_FGO_CORE_GRAPHSENSORCENTRIC_H
