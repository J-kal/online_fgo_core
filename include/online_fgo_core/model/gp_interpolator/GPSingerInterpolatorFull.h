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

#ifndef ONLINE_FGO_GPSingerINTERPOLATORPOSE3FULL_H
#define ONLINE_FGO_GPSingerINTERPOLATORPOSE3FULL_H

#pragma once

#include "GPWNOJInterpolatorFull.h"


namespace fgo::models {
  class GPSingerInterpolatorFull : public GPWNOJInterpolatorFull { //Translation world/earth Rotation body

  private:
    typedef GPSingerInterpolatorFull This;

  public:
    /// Default constructor: only for serialization
    GPSingerInterpolatorFull() : GPWNOJInterpolatorFull() {}

    /**
     * Constructor
     * @param Qc noise model of Qc
     * @param Ad for singer interpolation
     * @param delta_t the time between the two states
     * @param tau the time of interval status
     */
    explicit GPSingerInterpolatorFull(const gtsam::SharedNoiseModel &Qc_model, const gtsam::Matrix6 Ad,
                                      double delta_t = 0.0, double tau = 0.0, bool useAutoDiff = false,
                                      bool calcJacobian = true) :
      GPWNOJInterpolatorFull(Qc_model, delta_t, tau, useAutoDiff, calcJacobian, false) {
      // Calcuate Lambda and Psi
      Lambda_ = fgo::utils::calcLambdaSinger(Qc_, Ad_, delta_t_, tau_);
      Psi_ = fgo::utils::calcPsiSinger(Qc_, Ad_, delta_t_, tau_);
    }

    GPSingerInterpolatorFull(const This &interpolator) :
      GPWNOJInterpolatorFull(interpolator) {
      Lambda_ = fgo::utils::calcLambdaSinger(Qc_, Ad_, delta_t_, tau_);
      Psi_ = fgo::utils::calcPsiSinger(Qc_, Ad_, delta_t_, tau_);
      useAutoDiff_ = interpolator.useAutoDiff_;
      calcJacobian_ = interpolator.calcJacobian_;
    }

    /** Virtual destructor */
    ~GPSingerInterpolatorFull() override = default;

    void recalculate(const double &delta_t, const double &tau, const gtsam::Matrix66 &Ad,
                     const gtsam::Vector6 &accI, const gtsam::Vector6 &accJ) override {

      update(delta_t, tau, Ad);
      Lambda_ = fgo::utils::calcLambdaSinger(Qc_, Ad_, delta_t, tau);
      Psi_ = fgo::utils::calcPsiSinger(Qc_, Ad_, delta_t, tau);

      accI_ = accI;
      accJ_ = accJ;
    }

    /**
    * Testables
    */

    /** equals specialized to this factor */
    [[nodiscard]] virtual bool equals(const This &expected, double tol = 1e-9) const {
      return fabs(this->delta_t_ - expected.delta_t_) < tol &&
             fabs(this->tau_ - expected.tau_) < tol &&
             gtsam::equal_with_abs_tol(this->Qc_, expected.Qc_, tol) &&
             gtsam::equal_with_abs_tol(this->Ad_, expected.Ad_) &&
             gtsam::equal_with_abs_tol(this->Lambda_, expected.Lambda_, tol) &&
             gtsam::equal_with_abs_tol(this->Psi_, expected.Psi_, tol);
    }

    /** print contents */
    void print(const std::string &s = "") const override {
      std::cout << s << "GPSingerInterpolatorFull" << std::endl;
      std::cout << "delta_t = " << delta_t_ << ", tau = " << tau_ << std::endl;
      //std::cout << "Qc = " << Qc_ << std::endl;
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;

    template<class ARCHIVE>
    void serialize(ARCHIVE &ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_NVP(delta_t_);
      ar & BOOST_SERIALIZATION_NVP(tau_);
      using namespace boost::serialization;
      ar & make_nvp("Qc", make_array(Qc_.data(), Qc_.size()));
      ar & make_nvp("Ad", make_array(Ad_.data(), Ad_.size()));
      ar & make_nvp("Lambda", make_array(Lambda_.data(), Lambda_.size()));
      ar & make_nvp("Psi", make_array(Psi_.data(), Psi_.size()));
    }
  };
}

/// traits
namespace gtsam {
  template<>
  struct traits<fgo::models::GPSingerInterpolatorFull> : public Testable<
    fgo::models::GPSingerInterpolatorFull> {
  };
}

#endif //ONLINE_FGO_GPWNOJINTERPOLATOR_H
