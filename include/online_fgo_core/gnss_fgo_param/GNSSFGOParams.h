// Copyright 2021 Institute of Automatic Control RWTH Aachen University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and limitations under the License.
//
// Author: Haoming Zhang (h.zhang@irt.rwth-aachen.de)
//

#ifndef GNSS_FGO_ONLINEFGOPARAMS_H
#define GNSS_FGO_ONLINEFGOPARAMS_H

#pragma once

#include <string>
#include "../data/DataTypesFGO.h"

namespace gnss_fgo {
  struct GNSSFGOParams {
    bool offlineProcess = false;
    bool verbose = false;
    bool useIMUAsTimeReference = false;
    int optFrequency = 10; // 10 Hz
    int stateFrequency = 10;
    int bufferSize = 100;

    bool calibGravity = true;
    bool calcErrorOnOpt = false;
    bool delayFromPPS = false;
    double pvtMeasTimeOffset = 0.;

    bool imuMessageHasUncertainty = false;
    double accelerometerSigma = 0.0008;
    double integrationSigma = 1e-4;
    double gyroscopeSigma = 0.00052;
    double biasAccSigma = 0.0004;
    double biasOmegaSigma = 0.00087;
    double biasAccOmegaInt = 0.00001;
    uint IMUMeasurementFrequency = 100;  // 10 Hz

    bool addGPPriorFactor = false;
    bool addGPInterpolatedFactor = false;
    fgo::data::GPModelType gpType = fgo::data::GPModelType::WNOA;

    bool UsePPSTimeSync = true;

    // Initializer
    bool initGyroBiasAsZero = true;
    bool cleanIMUonInit = true;
    bool useHeaderTimestamp = true;

    virtual ~GNSSFGOParams()
    = default;
  };

  typedef std::shared_ptr<GNSSFGOParams> GNSSFGOParamsPtr;
}

#endif //GNSS_FGO_ONLINEFGOPARAMS_H
