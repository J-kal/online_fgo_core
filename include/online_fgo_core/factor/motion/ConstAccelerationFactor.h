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


#ifndef ONLINE_FGO_CONSTACCELERATIONRATEFACTOR_H
#define ONLINE_FGO_CONSTACCELERATIONRATEFACTOR_H

#pragma once

#include <cmath>
#include <fstream>
#include <iostream>

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/base/numericalDerivative.h>

#include "../FactorTypeID.h"

namespace fgo::factor {
/**
 * 3-way angular rate factor, including pose and angular rate in reference ecef frame, imu bias.
 */
  class ConstAccelerationFactor : public gtsam::NoiseModelFactor2<gtsam::Vector6, gtsam::imuBias::ConstantBias> {
  private:
    gtsam::Vector6 accMeasured_;
    bool useAutoDiff_ = true;
    typedef ConstAccelerationFactor This;
    typedef gtsam::NoiseModelFactor2<gtsam::Vector6, gtsam::imuBias::ConstantBias> Base;

  public:

    ConstAccelerationFactor() = default;  /* Default constructor */

    /**
     * @param betweenMeasured odometry measurement [dx dy dtheta] in body frame
     */
    ConstAccelerationFactor(gtsam::Key accKey, gtsam::Key biasKey,
                            const gtsam::Vector6 &accMeasured,
                            const gtsam::SharedNoiseModel &model, bool useAutoDiff = true)
      : Base(model, accKey, biasKey),
        accMeasured_(accMeasured),
        useAutoDiff_(useAutoDiff) {
    }

    ~ConstAccelerationFactor() override = default;

    /// @return a deep copy of this factor
    [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    [[nodiscard]] gtsam::Vector evaluateError(const gtsam::Vector6 &acc,
                                              const gtsam::imuBias::ConstantBias &bias,
                                              boost::optional<gtsam::Matrix &> H1 = boost::none,
                                              boost::optional<gtsam::Matrix &> H2 = boost::none) const override {
      if (useAutoDiff_) {
        if (H1)
          *H1 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::Vector6>(
            boost::bind(&This::evaluateError_, this, boost::placeholders::_1, bias), acc);
        if (H2)
          *H2 = gtsam::numericalDerivative11<gtsam::Vector6, gtsam::imuBias::ConstantBias>(
            boost::bind(&This::evaluateError_, this, acc, boost::placeholders::_1), bias);
        return evaluateError_(acc, bias);
      } else {
        gtsam::Matrix HaccBias;
        const auto unbiasedAccLin = bias.correctAccelerometer(accMeasured_.tail(3), H2 ? &HaccBias : 0);

        if (H1) *H1 = gtsam::I_6x6;
        if (H2) *H2 = -HaccBias;
        return acc - (gtsam::Vector6() << accMeasured_.head(3), unbiasedAccLin).finished();
      }
    }

    [[nodiscard]] gtsam::Vector6
    evaluateError_(const gtsam::Vector6 &acc, const gtsam::imuBias::ConstantBias &bias) const {
      const auto unbiasedAccLin = bias.correctAccelerometer(accMeasured_.tail(3));
      return acc - (gtsam::Vector6() << accMeasured_.head(3), unbiasedAccLin).finished();
    }

    /** lifting all related state values in a vector after the ordering for evaluateError **/
    gtsam::Vector liftValuesAsVector(const gtsam::Values &values) {
      const auto acc = values.at<gtsam::Vector6>(key1());
      const auto bias = values.at<gtsam::imuBias::ConstantBias>(key2());
      return (gtsam::Vector(12) << acc, bias.vector()).finished();
    }

    gtsam::Values generateValuesFromStateVector(const gtsam::Vector &state) {
      assert(state.size() != 6);
      gtsam::Values values;
      try {
        values.insert(key1(), gtsam::Vector6(state.block<6, 1>(0, 0)));
        values.insert(key2(), gtsam::imuBias::ConstantBias(state.block<6, 1>(6, 0)));
      }
      catch (std::exception &ex) {
        std::cout << "Factor " << "this factor" << " cannot generate values from state vector " << state << " due to "
                  << ex.what() << std::endl;
      }
      return values;
    }

    /** return the measured */
    [[nodiscard]] const gtsam::Vector6 &measured() const {
      return (gtsam::Vector6() << accMeasured_).finished();
    }

    /** equals specialized to this factor */
    [[nodiscard]] bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
      const This *e = dynamic_cast<const This *> (&expected);
      return e != nullptr && Base::equals(*e, tol)
             && gtsam::equal_with_abs_tol(accMeasured_, e->accMeasured_, tol);
    }

    /** print contents */
    void print(const std::string &s = "",
               const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
      std::cout << s << "ConstAccelerationFactor" << std::endl;
      Base::print("", keyFormatter);
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;

    template<class ARCHIVE>
    void serialize(ARCHIVE &ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("ConstAccelerationFactor",
                                          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(accMeasured_);
    }
  }; // PrDrFactor
} // namespace fgonav

/// traits
namespace gtsam {
  template<>
  struct traits<fgo::factor::ConstAccelerationFactor> : public Testable<fgo::factor::ConstAccelerationFactor> {
  };
}


#endif //ONLINE_FGO_CONSTACCELERATIONRATEFACTOR_H

