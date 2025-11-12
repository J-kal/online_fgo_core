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
//  Author: Zirui Bai (zirui.bai@rwth-aachen.de)
//          Haoming Zhang (haoming.zhang@rwth-aachen.de)
//
//

/*
 *
 */


#ifndef ONLINE_FGO_GPSINGERPRIORPOSE3FULL
#define ONLINE_FGO_GPSINGERPRIORPOSE3FULL
#pragma once

#include "GPPriorBase.h"

namespace fgo::factor {
  class GPSingerPriorFull : public GPPriorBase,
                            public fgo::NoiseModelFactor8<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6,
                                   gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6> {
  private:
    typedef GPSingerPriorFull This;
    typedef fgo::NoiseModelFactor8<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6,
      gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6> Base;

  public:
    GPSingerPriorFull() = default;

    GPSingerPriorFull(
      gtsam::Key poseKey1, gtsam::Key velKey1, gtsam::Key omegaKey1, gtsam::Key acc1,
      gtsam::Key poseKey2, gtsam::Key velKey2, gtsam::Key omegaKey2, gtsam::Key acc2,
      double dt, const gtsam::SharedNoiseModel &Qc_model, const gtsam::Matrix66 &ad,
      bool useAutoDiff = false, bool calcJacobian = true,
      const gtsam::Vector6 &accI = gtsam::Vector6(),
      const gtsam::Vector6 &accJ = gtsam::Vector6()) :
      GPPriorBase(dt, ad, accI, accJ, useAutoDiff, calcJacobian),
      Base(gtsam::noiseModel::Gaussian::Covariance(fgo::utils::calcQ<6>(fgo::utils::getQc(Qc_model), ad, dt)), poseKey1,
           velKey1, omegaKey1, acc1, poseKey2, velKey2, omegaKey2, acc2) {
    }

    ~GPSingerPriorFull() override = default;

    /// @return a deep copy of this factor
    [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }


    /// factor error function
    [[nodiscard]] gtsam::Vector evaluateError(
      const gtsam::Pose3 &pose1, const gtsam::Vector3 &vel1, const gtsam::Vector3 &omega1,
      const gtsam::Vector6 &acc1,
      const gtsam::Pose3 &pose2, const gtsam::Vector3 &vel2, const gtsam::Vector3 &omega2,
      const gtsam::Vector6 &acc2,
      boost::optional<gtsam::Matrix &> H1 = boost::none,
      boost::optional<gtsam::Matrix &> H2 = boost::none,
      boost::optional<gtsam::Matrix &> H3 = boost::none,
      boost::optional<gtsam::Matrix &> H4 = boost::none,
      boost::optional<gtsam::Matrix &> H5 = boost::none,
      boost::optional<gtsam::Matrix &> H6 = boost::none,
      boost::optional<gtsam::Matrix &> H7 = boost::none,
      boost::optional<gtsam::Matrix &> H8 = boost::none) const override {

      using namespace gtsam;
      using namespace fgo::utils;
      if (useAutoDiff_) {
        // jacobians
        if (H1) {
          *H1 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
            std::bind(&This::evaluateError_, this, std::placeholders::_1, vel1, omega1, acc1, pose2, vel2,
                      omega2, acc2), pose1, 1e-5);
        }
        if (H2) {
          *H2 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector3>(
            std::bind(&This::evaluateError_, this, pose1, std::placeholders::_1, omega1, acc1, pose2, vel2,
                      omega2, acc2), vel1, 1e-5);
        }
        if (H3) {
          *H3 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector3>(
            std::bind(&This::evaluateError_, this, pose1, vel1, std::placeholders::_1, acc1, pose2, vel2,
                      omega2, acc2), omega1, 1e-5);
        }

        if (H4) {
          *H4 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector6>(
            std::bind(&This::evaluateError_, this, pose1, vel1, omega1, std::placeholders::_1, pose2, vel2,
                      omega2, acc2), acc1, 1e-5);
        }

        if (H5) {
          *H5 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
            std::bind(&This::evaluateError_, this, pose1, vel1, omega1, acc1, std::placeholders::_1, vel2,
                      omega2, acc2), pose2, 1e-5);
        }

        if (H6) {
          *H6 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector3>(
            std::bind(&This::evaluateError_, this, pose1, vel1, omega1, acc1, pose2, std::placeholders::_1,
                      omega2, acc2), vel2, 1e-5);
        }

        if (H7) {
          *H7 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector3>(
            std::bind(&This::evaluateError_, this, pose1, vel1, omega1, acc1, pose2, vel2,
                      std::placeholders::_1, acc2), omega2, 1e-5);
        }

        if (H8) {
          *H8 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector6>(
            std::bind(&This::evaluateError_, this, pose1, vel1, omega1, acc1, pose2, vel2, omega2,
                      std::placeholders::_1), acc2, 1e-5);
        }

        return evaluateError_(pose1, vel1, omega1, acc1, pose2, vel2, omega2, acc2);
      } else {
        Matrix6 Hinv, Hcomp1, Hcomp2, Hlogmap;
        Vector6 r;
        if (H1 || H5)
          r = Pose3::Logmap(pose1.inverse(&Hinv).compose(pose2, &Hcomp1, &Hcomp2), &Hlogmap);
        else
          r = Pose3::Logmap(pose1.inverse().compose(pose2));

        Matrix6 Jinv = gtsam::I_6x6;
        if (calcJacobian_) {
          Jinv = fgo::utils::rightJacobianPose3inv(r);
        }

        Matrix63 H1v, H1w, H2v, H2w;
        Matrix6 H1p, H2p;
        Matrix_18_6 Hv1, Hv2, Hacc1, Hacc2;

        Vector6 v1, v2;

        if (H2 || H3 || H4 || H6 || H7 || H8) {
          v1 = convertVwWbToVbWb(vel1, omega1, pose1, &H1v, &H1w, &H1p);
          v2 = convertVwWbToVbWb(vel2, omega2, pose2, &H2v, &H2w, &H2p);
          Hv1 = (Matrix_18_6() << -delta_t_ * Matrix6::Identity(), -Matrix6::Identity(), Matrix6::Zero()).finished();
          Hacc1 = (Matrix_18_6() << -c1_, -c2_, -c3_).finished();
        } else {
          v1 = convertVwWbToVbWb(vel1, omega1, pose1);
          v2 = convertVwWbToVbWb(vel2, omega2, pose2);
        }

        const gtsam::Vector6 JinvVel2 = Jinv * v2;
        const gtsam::Vector3 omegaJinvVel2 = JinvVel2.head(3), rho2JinvVel2 = JinvVel2.tail(3);
        const gtsam::Matrix3 X = gtsam::skewSymmetric(omegaJinvVel2), Y = gtsam::skewSymmetric(rho2JinvVel2);
        const gtsam::Matrix6 JinvVel2SkewMatrix = (gtsam::Matrix6() << X, gtsam::Matrix3::Zero(), Y, X).finished();
        const gtsam::Vector3 omegaV2 = v2.head(3), rhoV2 = v2.tail(3);
        const gtsam::Matrix3 XV2 = gtsam::skewSymmetric(omegaV2), YV2 = gtsam::skewSymmetric(rhoV2);
        const gtsam::Matrix6 V2SkewMatrix = (gtsam::Matrix6() << XV2, gtsam::Matrix3::Zero(), YV2, XV2).finished();
        const gtsam::Matrix3 XA2 = gtsam::skewSymmetric(acc2.head(3)), YA2 = gtsam::skewSymmetric(acc2.tail(3));
        const gtsam::Matrix6 A2SkewMatrix = (gtsam::Matrix6() << XA2, gtsam::Matrix3::Zero(), YA2, XA2).finished();

        if (H7 || H8) {
          const gtsam::Vector6 JinvAccJ = Jinv * acc2;
          const gtsam::Vector3 omegaAccJ = JinvVel2.head(3), rhoAccJ = JinvVel2.tail(3);
          const gtsam::Matrix3 XAccJ = gtsam::skewSymmetric(omegaAccJ), YAccJ = gtsam::skewSymmetric(rhoAccJ);
          const gtsam::Matrix6 JinvAccJSkewMatrix = (gtsam::Matrix6()
            << XAccJ, gtsam::Matrix3::Zero(), YAccJ, XAccJ).finished();

          Hv2 = (Matrix_18_6() << Matrix6::Zero(),
            Jinv,
            -0.5 * JinvAccJSkewMatrix + 0.5 * V2SkewMatrix * Jinv).finished();

          Hacc2 = (Matrix_18_6() << Matrix6::Zero(), Matrix6::Zero(), Jinv).finished();
        }

        gtsam::Matrix6 JdiffV2 = gtsam::Matrix6::Zero();
        if ((H1 || H5) && calcJacobian_)
          JdiffV2 = jacobianMethodNumercialDiff(rightJacobianPose3inv, r, v2);

        if (H1) {
          gtsam::Matrix6 J_Ti = Hlogmap * Hcomp1 * Hinv;
          gtsam::Matrix6 Jdiff_Ti = gtsam::Matrix6::Zero();
          if (calcJacobian_) {
            Jdiff_Ti = JdiffV2 * J_Ti;
          }
          // JinvVel2SkewMatrix * v2 = - v2SkewMatrix * Jinv * V2
          *H1 = (fgo::utils::Matrix_18_6() << J_Ti - delta_t_ * H1p,  //
            Jdiff_Ti - H1p,
            (0.25 * V2SkewMatrix * V2SkewMatrix + 0.5 * A2SkewMatrix) * J_Ti).finished();
          // checked on 28th. Aug.
        }

        if (H2) *H2 = Hv1 * H1v;
        if (H3) *H3 = Hv1 * H1w;
        if (H4) *H4 = Hacc1;

        if (H5) {
          gtsam::Matrix6 J_Tj = Hlogmap * Hcomp2;
          gtsam::Matrix6 Jdiff_Ti = gtsam::Matrix6::Zero();
          if (calcJacobian_) {
            Jdiff_Ti = JdiffV2 * J_Tj;
          }
          *H5 = (fgo::utils::Matrix_18_6() << J_Tj,
            Jdiff_Ti + Jinv * H2p,
            (0.25 * V2SkewMatrix * V2SkewMatrix + 0.5 * A2SkewMatrix) * J_Tj).finished();
        }

        if (H6) *H6 = Hv2 * H2v;
        if (H7) *H7 = Hv2 * H2w;
        if (H8) *H8 = Hacc2;   // Jacobian for the linear acc2

        return (gtsam::Vector(18) << (r - v1 * delta_t_ - c1_ * acc1),
          Jinv * v2 - v1 - c2_ * acc1,
          -0.5 * JinvVel2SkewMatrix * v2 + Jinv * acc2 - c3_ * acc1).finished();
      }
    }

    [[nodiscard]] gtsam::Vector
    evaluateError_(const gtsam::Pose3 &pose1, const gtsam::Vector3 &vel1, const gtsam::Vector3 &omega1,
                   const gtsam::Vector6 &acc1,
                   const gtsam::Pose3 &pose2, const gtsam::Vector3 &vel2, const gtsam::Vector3 &omega2,
                   const gtsam::Vector6 &acc2) const {
      gtsam::Vector6 r = gtsam::Pose3::Logmap(pose1.inverse().compose(pose2));

      gtsam::Matrix6 Jinv = gtsam::I_6x6;

      gtsam::Vector6 v1 = fgo::utils::convertVwWbToVbWb(vel1, omega1, pose1);
      gtsam::Vector6 v2 = fgo::utils::convertVwWbToVbWb(vel2, omega2, pose2);

      if (calcJacobian_) {
        Jinv = fgo::utils::rightJacobianPose3inv(r);
      }

      const gtsam::Vector6 JinvVel2 = Jinv * v2;
      const gtsam::Vector3 omega = JinvVel2.head(3), rho = JinvVel2.tail(3);
      const gtsam::Matrix3 skew_v_omega = gtsam::skewSymmetric(omega), skew_vel_linear = gtsam::skewSymmetric(rho);
      // eq. (35) and (42) in https://arxiv.org/pdf/1809.06518.pdf
      const auto JinvVel2SkewMatrix = (gtsam::Matrix6()
        << skew_v_omega, gtsam::Matrix3::Zero(), skew_vel_linear, skew_v_omega).finished();
      // RA-L Tang et al. eq (41)
      //std::cout << JinvVel2SkewMatrix << std::endl;
      gtsam::Vector err = (gtsam::Vector(18) << (r - v1 * delta_t_ - c1_ * acc1),
        Jinv * v2 - v1 - c2_ * acc1,
        -0.5 * JinvVel2SkewMatrix * v2 + Jinv * acc2 - c3_ * acc1).finished();
      return err;
    }

    /** lifting all related state values in a vector after the ordering for evaluateError **/
    gtsam::Vector liftValuesAsVector(const gtsam::Values &values) {
      const auto poseI = values.at<gtsam::Pose3>(key1());
      const auto velI = values.at<gtsam::Vector3>(key2());
      const auto omegaI = values.at<gtsam::Vector3>(key3());
      const auto accI = values.at<gtsam::Vector6>(key4());
      const auto poseJ = values.at<gtsam::Pose3>(key5());
      const auto velJ = values.at<gtsam::Vector3>(key6());
      const auto omegaJ = values.at<gtsam::Vector3>(key7());
      const auto accJ = values.at<gtsam::Vector6>(key8());

      const auto liftedStates = (gtsam::Vector(36) << poseI.rotation().rpy(),
        poseI.translation(),
        velI, omegaI, accI,
        poseJ.rotation().rpy(),
        poseJ.translation(),
        velJ, omegaJ, accJ).finished();
      return liftedStates;
    }

    gtsam::Values generateValuesFromStateVector(const gtsam::Vector &state) {
      assert(state.size() != 36);
      gtsam::Values values;
      try {
        values.insert(key1(), gtsam::Pose3(gtsam::Rot3::RzRyRx(state.block<3, 1>(0, 0)),
                                           gtsam::Point3(state.block<3, 1>(3, 0))));
        values.insert(key2(), gtsam::Vector3(state.block<3, 1>(6, 0)));
        values.insert(key3(), gtsam::Vector3(state.block<3, 1>(9, 0)));
        values.insert(key4(), gtsam::Vector6(state.block<6, 1>(12, 0)));

        values.insert(key5(), gtsam::Pose3(gtsam::Rot3::RzRyRx(state.block<3, 1>(18, 0)),
                                           gtsam::Point3(state.block<3, 1>(21, 0))));
        values.insert(key6(), gtsam::Vector3(state.block<3, 1>(24, 0)));
        values.insert(key7(), gtsam::Vector3(state.block<3, 1>(27, 0)));
        values.insert(key8(), gtsam::Vector6(state.block<6, 1>(30, 0)));
      }
      catch (std::exception &ex) {
        std::cout << "Factor " << "this factor" << " cannot generate values from state vector " << state << " due to "
                  << ex.what() << std::endl;
      }
      return values;
    }


    /** equals specialized to this factor */
    [[nodiscard]] bool equals(const gtsam::NonlinearFactor &expected, double tol = 1e-9) const override {
      const This *e = dynamic_cast<const This *> (&expected);
      return e != nullptr && Base::equals(*e, tol) && fabs(this->delta_t_ - e->delta_t_) < tol;
    }

    /** print contents */
    void print(const std::string &s = "",
               const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override {
      std::cout << s << "4-way Gaussian Process Factor Pose3 VW" << std::endl;
      Base::print("", keyFormatter);
    }

  private:
    /** Serialization function */
    //friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE &ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      ar & BOOST_SERIALIZATION_NVP(delta_t_);
    }
  }; // GaussianProcessPriorPose3
} // namespace fgo


/// traits
namespace gtsam {
  template<>
  struct traits<fgo::factor::GPSingerPriorFull> : public Testable<fgo::factor::GPSingerPriorFull> {
  };
}


#endif

