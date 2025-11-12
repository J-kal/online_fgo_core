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
//  Author: Haoming Zhang (haoming.zhang@rwth-aachen.de)
//
//

//
// Created by haoming on 30.04.24.
//

#ifndef ONLINE_FGO_GPPRIORBASE_H
#define ONLINE_FGO_GPPRIORBASE_H

#pragma once

#include <ostream>
#include <boost/lexical_cast.hpp>

#include <gtsam/geometry/concepts.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/numericalDerivative.h>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

#include "../FactorType.h"
#include "../../utils/GPUtils.h"
#include "../../utils/Pose3Utils.h"
#include "../FactorTypeID.h"

namespace fgo::factor {

  class GPPriorBase {
  protected:
    double delta_t_{};
    bool useAutoDiff_ = false;
    bool calcJacobian_ = false;
    gtsam::Vector6 accI_;   // als Reference
    gtsam::Vector6 accJ_;
    gtsam::Matrix6 ad_;
    gtsam::Matrix66 c1exp_, c2exp_, c3exp_, c1_, c2_, c3_;


  public:
    explicit GPPriorBase(double dt,
                         bool useAutoDiff = false, bool calcJacobian = true) :
      delta_t_(dt), useAutoDiff_(useAutoDiff), calcJacobian_(calcJacobian) {};

    explicit GPPriorBase(double dt, const gtsam::Vector6 &accI, const gtsam::Vector6 &accJ,
                         bool useAutoDiff = false, bool calcJacobian = true) :
      delta_t_(dt), useAutoDiff_(useAutoDiff), calcJacobian_(calcJacobian), accI_(accI), accJ_(accJ) {};

    explicit GPPriorBase(double dt, const gtsam::Matrix66 &ad, const gtsam::Vector6 &accI, const gtsam::Vector6 &accJ,
                         bool useAutoDiff = false, bool calcJacobian = true) :
      delta_t_(dt), useAutoDiff_(useAutoDiff), calcJacobian_(calcJacobian), accI_(accI), accJ_(accJ), ad_(ad) {
      c1exp_ = (gtsam::Matrix66() << -gtsam::Matrix6::Identity() * ad_ * delta_t_).finished();
      c1_ = (gtsam::Matrix66()
        << ad_.inverse() * ad_.inverse() * ((ad_ * delta_t_) - gtsam::Matrix6::Identity() + c1exp_.exp())).finished();
      c2exp_ = (gtsam::Matrix66() << -gtsam::Matrix6::Identity() * ad_ * delta_t_).finished();
      c2_ = (gtsam::Matrix66() << ad_.inverse() * (gtsam::Matrix6::Identity() - c2exp_.exp())).finished();
      c3exp_ = (gtsam::Matrix66() << -gtsam::Matrix6::Identity() * ad_ * delta_t_).finished();
      c3_ = (gtsam::Matrix66() << c3exp_.exp()).finished();
    }

    GPPriorBase() = default;

    ~GPPriorBase() = default;

    void updateMeasuredAcceleration(const gtsam::Vector6 &accI, const gtsam::Vector6 &accJ) {
      accI_ = accI;
      accJ_ = accJ;
    }

    gtsam::Vector6 getMeasuredAccelerationI() const { return accI_; };

    gtsam::Vector6 getMeasuredAccelerationJ() const { return accJ_; };

    void updateAdMatrix(const gtsam::Matrix66 &ad) { ad_ = ad; };

  };


}


#endif //ONLINE_FGO_GPPRIORBASE_H

