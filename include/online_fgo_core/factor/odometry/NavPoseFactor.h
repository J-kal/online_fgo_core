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
//

#ifndef ONLINE_FGO_NAVPOSEFACTOR_H
#define ONLINE_FGO_NAVPOSEFACTOR_H

#pragma once

#include <utility>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include "../FactorType.h"
#include "../../utils/NavigationTools.h"
#include "../FactorTypeID.h"

namespace fgo::factor {
  class NavPoseFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
  protected:
    gtsam::Pose3 poseMeasured_;

    typedef NavPoseFactor This;
    typedef gtsam::NoiseModelFactor1<gtsam::Pose3> Base;

  public:
    NavPoseFactor() = default;

    NavPoseFactor(gtsam::Key poseKey, const gtsam::Pose3 &poseMeasured,
                  const gtsam::SharedNoiseModel &model) :
      Base(model, poseKey), poseMeasured_(poseMeasured) {
    }

    ~NavPoseFactor() override = default;

    /// @return a deep copy of this factor
    [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    /** factor error */
    [[nodiscard]] gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                                              boost::optional<gtsam::Matrix &> H1 = boost::none) const override {
      return pose.localCoordinates(poseMeasured_, H1);
    }

    /** lifting all related state values in a vector after the ordering for evaluateError **/
    gtsam::Vector liftValuesAsVector(const gtsam::Values &values) {
      const auto poseI = values.at<gtsam::Pose3>(key());

      const auto liftedStates = (gtsam::Vector(6) <<
                                                  poseI.rotation().rpy(),
        poseI.translation()).finished();
      return liftedStates;
    }

    gtsam::Values generateValuesFromStateVector(const gtsam::Vector &state) {
      assert(state.size() != 6);
      gtsam::Values values;
      try {
        values.insert(key(), gtsam::Pose3(gtsam::Rot3::RzRyRx(state.block<3, 1>(0, 0)),
                                          gtsam::Point3(state.block<3, 1>(3, 0))));
      }
      catch (std::exception &ex) {
        std::cout << "Factor " << "this factor" << " cannot generate values from state vector " << state << " due to "
                  << ex.what() << std::endl;
      }
      return values;
    }

    /** return the measured */
    [[nodiscard]] gtsam::Pose3 measured() const {
      return poseMeasured_;
    }

    /** equals specialized to this factor */
    [[nodiscard]] bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
      const This *e = dynamic_cast<const This *> (&expected);
      return e != nullptr && Base::equals(*e, tol)
             && gtsam::equal_with_abs_tol(this->poseMeasured_.translation(),
                                          e->poseMeasured_.translation(), tol);
    }

    /** print contents */
    void print(const std::string &s = "",
               const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
      std::cout << s << "NavPoseFactor" << std::endl;
      Base::print("", keyFormatter);
    }

  private:
    /** Serialization function */
    friend class boost::serialization::access;

    template<class ARCHIVE>
    void serialize(ARCHIVE &ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("NavPoseFactor",
                                          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(poseMeasured_);
    }


  };
}
/// traits
namespace gtsam {
  template<>
  struct traits<fgo::factor::NavPoseFactor> :
    public Testable<fgo::factor::NavPoseFactor> {
  };
}
#endif //ONLINE_FGO_NAVPOSEFACTOR_H

