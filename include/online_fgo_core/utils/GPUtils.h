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

#ifndef ONLINE_FGO_GPUTILS_H
#define ONLINE_FGO_GPUTILS_H

#pragma once

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>

#include <cmath>

namespace fgo::utils {
/// get Qc covariance matrix from noise model
  inline gtsam::Matrix getQc(const gtsam::SharedNoiseModel &Qc_model) {
    auto *Gassian_model = dynamic_cast<gtsam::noiseModel::Gaussian *>(Qc_model.get());
    return (Gassian_model->R().transpose() * Gassian_model->R()).inverse();  // => R().transpose() * R() = inv(sigma)
  }

/// calculate Q
  template<int Dim>
  Eigen::Matrix<double, 2 * Dim, 2 * Dim> calcQ(const Eigen::Matrix<double, Dim, Dim> &Qc, double tau) {
    Eigen::Matrix<double, 2 * Dim, 2 * Dim> Q = (Eigen::Matrix<double, 2 * Dim, 2 * Dim>() <<
                                                                                           1.0 / 3 * pow(tau, 3.0) * Qc,
      1.0 / 2 * pow(tau, 2.0) * Qc,
      1.0 / 2 * pow(tau, 2.0) * Qc, tau * Qc).finished();
    return Q;
  }

/// calculate Q_inv
  template<int Dim>
  Eigen::Matrix<double, 2 * Dim, 2 * Dim> calcQ_inv(const Eigen::Matrix<double, Dim, Dim> &Qc, double tau) {
    Eigen::Matrix<double, Dim, Dim> Qc_inv = Qc.inverse();
    Eigen::Matrix<double, 2 * Dim, 2 * Dim> Q_inv =
      (Eigen::Matrix<double, 2 * Dim, 2 * Dim>() <<
                                                 12.0 * pow(tau, -3.0) * Qc_inv, (-6.0) * pow(tau, -2.0) * Qc_inv,
        (-6.0) * pow(tau, -2.0) * Qc_inv, 4.0 * pow(tau, -1.0) * Qc_inv).finished();
    return Q_inv;
  }

/// calculate Phi
  template<int Dim>
  Eigen::Matrix<double, 2 * Dim, 2 * Dim> calcPhi(double tau) {

    Eigen::Matrix<double, 2 * Dim, 2 * Dim> Phi = (Eigen::Matrix<double, 2 * Dim, 2 * Dim>() <<
                                                                                             Eigen::Matrix<double, Dim, Dim>::Identity(),
      tau * Eigen::Matrix<double, Dim, Dim>::Identity(),
      Eigen::Matrix<double, Dim, Dim>::Zero(), Eigen::Matrix<double, Dim, Dim>::Identity()).finished();
    return Phi;
  }

/// calculate Lambda
  template<int Dim>
  Eigen::Matrix<double, 2 * Dim, 2 * Dim> calcLambda(const Eigen::Matrix<double, Dim, Dim> &Qc,
                                                     double delta_t, const double tau) {

    Eigen::Matrix<double, 2 * Dim, 2 * Dim> Lambda =
      calcPhi<Dim>(tau) - calcQ(Qc, tau) * (calcPhi<Dim>(delta_t - tau).transpose())
                          * calcQ_inv(Qc, delta_t) * calcPhi<Dim>(delta_t);
    return Lambda;
  }

/// calculate Psi
  template<int Dim>
  Eigen::Matrix<double, 2 * Dim, 2 * Dim> calcPsi(const Eigen::Matrix<double, Dim, Dim> &Qc,
                                                  double delta_t, double tau) {

    Eigen::Matrix<double, 2 * Dim, 2 * Dim> Psi = calcQ(Qc, tau) * (calcPhi<Dim>(delta_t - tau).transpose())
                                                  * calcQ_inv(Qc, delta_t);
    return Psi;
  }

  ///WNOJ addition
  /// calculate Q
  template<int Dim>
  Eigen::Matrix<double, 3 * Dim, 3 * Dim> calcQ3(const Eigen::Matrix<double, Dim, Dim> &Qc, double tau) {
    Eigen::Matrix<double, 3 * Dim, 3 * Dim> Q = (Eigen::Matrix<double, 3 * Dim, 3 * Dim>() <<
                                                                                           1.0 / 20 * pow(tau, 5.0) *
                                                                                           Qc, 1.0 / 8 * pow(tau, 4.0) *
                                                                                               Qc, 1.0 / 6 *
                                                                                                   pow(tau, 3.0) * Qc,
      1.0 / 8 * pow(tau, 4.0) * Qc, 1.0 / 3 * pow(tau, 3.0) * Qc, 1.0 / 2 * pow(tau, 2.0) * Qc,
      1.0 / 6 * pow(tau, 3.0) * Qc, 1.0 / 2 * pow(tau, 2.0) * Qc, tau * Qc).finished();
    return Q;
  }

  /// calculate Q
  template<int Dim>
  Eigen::Matrix<double, 2 * Dim, 2 * Dim> calcQ3_12x12(const Eigen::Matrix<double, Dim, Dim> &Qc, double tau) {
    Eigen::Matrix<double, 2 * Dim, 2 * Dim> Q = (Eigen::Matrix<double, 2 * Dim, 2 * Dim>() <<
                                                                                           1.0 / 20 * pow(tau, 5.0) *
                                                                                           Qc, 1.0 / 8 * pow(tau, 4.0) *
                                                                                               Qc,
      1.0 / 8 * pow(tau, 4.0) * Qc, 1.0 / 3 * pow(tau, 3.0) * Qc).finished();
    return Q;
  }

  /// calculate Q_inv
  template<int Dim>
  Eigen::Matrix<double, 3 * Dim, 3 * Dim> calcQ_inv3(const Eigen::Matrix<double, Dim, Dim> &Qc, double tau) {
    Eigen::Matrix<double, Dim, Dim> Qc_inv = Qc.inverse();
    Eigen::Matrix<double, 3 * Dim, 3 * Dim> Q_inv = (Eigen::Matrix<double, 3 * Dim, 3 * Dim>() <<
                                                                                               720.0 * pow(tau, -5.0) *
                                                                                               Qc_inv, (-360.0) *
                                                                                                       pow(tau, -4.0) *
                                                                                                       Qc_inv, 60.0 *
                                                                                                               pow(tau,
                                                                                                                   -3.0) *
                                                                                                               Qc_inv,
      (-360.0) * pow(tau, -4.0) * Qc_inv, 192 * pow(tau, -3.0) * Qc_inv, (-36.0) * pow(tau, -2.0) * Qc_inv,
      60.0 * pow(tau, -3.0) * Qc_inv, (-36.0) * pow(tau, -2.0) * Qc_inv, 9.0 * pow(tau, -1.0) * Qc_inv).finished();
    return Q_inv;
  }

  /// calculate Phi
  template<int Dim>
  Eigen::Matrix<double, 3 * Dim, 3 * Dim> calcPhi3(double tau) {

    Eigen::Matrix<double, 3 * Dim, 3 * Dim> Phi = (Eigen::Matrix<double, 3 * Dim, 3 * Dim>() <<
                                                                                             Eigen::Matrix<double, Dim, Dim>::Identity(),
      tau * Eigen::Matrix<double, Dim, Dim>::Identity(), 0.5 * pow(tau, 2.0) *
                                                         Eigen::Matrix<double, Dim, Dim>::Identity(),
      Eigen::Matrix<double, Dim, Dim>::Zero(), Eigen::Matrix<double, Dim, Dim>::Identity(), tau *
                                                                                            Eigen::Matrix<double, Dim, Dim>::Identity(),
      Eigen::Matrix<double, Dim, Dim>::Zero(), Eigen::Matrix<double, Dim, Dim>::Zero(), Eigen::Matrix<double, Dim, Dim>::Identity()).finished();
    return Phi;
  }

  /// calculate Lambda
  template<int Dim>
  Eigen::Matrix<double, 3 * Dim, 3 * Dim> calcLambda3(const Eigen::Matrix<double, Dim, Dim> &Qc,
                                                      double delta_t, const double tau) {

    Eigen::Matrix<double, 3 * Dim, 3 * Dim> Lambda =
      calcPhi3<Dim>(tau) - calcQ3(Qc, tau) * (calcPhi3<Dim>(delta_t - tau).transpose())
                           * calcQ_inv3(Qc, delta_t) * calcPhi3<Dim>(delta_t);

    ///Eigen::Matrix<double, 3*Dim, 3*Dim> Lambda = calcPhi3<Dim>(tau) - calcPsi3(Qc, delta_t, tau) * (calcPhi3<Dim>(delta_t));
    // Eigen::Matrix<double, 2*Dim, 2*Dim> Lambda = calcPhi<Dim>(tau) - calcQ(Qc, tau) * (calcPhi<Dim>(delta_t - tau).transpose())
    //                                                                         * calcQ_inv(Qc, delta_t) * calcPhi<Dim>(delta_t);

    return Lambda;
  }

/// calculate Psi
  template<int Dim>
  Eigen::Matrix<double, 3 * Dim, 3 * Dim> calcPsi3(const Eigen::Matrix<double, Dim, Dim> &Qc,
                                                   double delta_t, double tau) {

    Eigen::Matrix<double, 3 * Dim, 3 * Dim> Psi = calcQ3(Qc, tau) * (calcPhi3<Dim>(delta_t - tau).transpose())
                                                  * calcQ_inv3(Qc, delta_t);

    //Eigen::Matrix<double, 3*Dim, 3*Dim> Psi = calcQ3(Qc, tau) * (calcPhi3<Dim>(delta_t).transpose())
    //                                        * calcQ_inv3(Qc, tau);
    return Psi;
  }

  ///WNOJ end
  ///Singer addition
  /// calculate Q, see original implementation in STEAM: https://github.com/utiasASRL/steam.git
  template<int Dim>
  Eigen::Matrix<double, 3 * Dim, 3 * Dim> calcQ(
    const Eigen::Matrix<double, Dim, Dim> &Qc,
    const Eigen::Matrix<double, Dim, Dim> &Ad,
    double dt) {
    /*Assume Ad = diag(ad,ad,ad,...,ad)*/
    Eigen::Matrix<double, 18, 18> Q = Eigen::Matrix<double, 18, 18>::Zero();
    for (int i = 0; i < 6; ++i) {
      const double ad = Ad(i, i);
      const double qc = Qc(i, i);
      if (fabs(ad) >= 1.0) {
        const double adi = 1.0 / ad;
        const double adi2 = adi * adi;
        const double adi3 = adi * adi2;
        const double adi4 = adi * adi3;
        const double adi5 = adi * adi4;
        const double adt = ad * dt;
        const double adt2 = adt * adt;
        const double adt3 = adt2 * adt;
        const double expon = std::exp(-adt);
        const double expon2 = std::exp(-2 * adt);
        Q(i, i) = qc * (
          0.5
          * adi5
          * (1 - expon2 + 2 * adt + (2.0 / 3.0) * adt3 - 2 * adt2 - 4 * adt * expon)
        );
        Q(i, i + 6) = Q(i + 6, i) = qc * (
          0.5 * adi4 * (expon2 + 1 - 2 * expon + 2 * adt * expon - 2 * adt + adt2)
        );
        Q(i, i + 12) = Q(i + 12, i) = qc * 0.5 * adi3 * (1 - expon2 - 2 * adt * expon);
        Q(i + 6, i + 6) = qc * 0.5 * adi3 * (4 * expon - 3 - expon2 + 2 * adt);
        Q(i + 6, i + 12) = Q(i + 12, i + 6) = qc * 0.5 * adi2 * (expon2 + 1 - 2 * expon);
        Q(i + 12, i + 12) = qc * 0.5 * adi * (1 - expon2);
      } else {
        const double dt2 = dt * dt;
        const double dt3 = dt * dt2;
        const double dt4 = dt * dt3;
        const double dt5 = dt * dt4;
        const double dt6 = dt * dt5;
        const double dt7 = dt * dt6;
        const double dt8 = dt * dt7;
        const double dt9 = dt * dt8;
        const double ad2 = ad * ad;
        const double ad3 = ad * ad2;
        const double ad4 = ad * ad3;
        // use Taylor series expansion about ad = 0
        Q(i, i) = qc * (
          0.05 * dt5
          - 0.0277778 * dt6 * ad
          + 0.00992063 * dt7 * ad2
          - 0.00277778 * dt8 * ad3
          + 0.00065586 * dt9 * ad4
        );
        Q(i, i + 6) = Q(i + 6, i) = qc * (
          0.125 * dt4
          - 0.0833333 * dt5 * ad
          + 0.0347222 * dt6 * ad2
          - 0.0111111 * dt7 * ad3
          + 0.00295139 * dt8 * ad4
        );
        Q(i, i + 12) = Q(i + 12, i) = qc * (
          (1 / 6) * dt3
          - (1 / 6) * dt4 * ad
          + 0.0916667 * dt5 * ad2
          - 0.0361111 * dt6 * ad3
          + 0.0113095 * dt7 * ad4
        );
        Q(i + 6, i + 6) = qc * (
          (1 / 3) * dt3
          - 0.25 * dt4 * ad
          + 0.116667 * dt5 * ad2
          - 0.0416667 * dt6 * ad3
          + 0.0123016 * dt7 * ad4
        );
        Q(i + 6, i + 12) = Q(i + 12, i + 6) = qc * (
          0.5 * dt2
          - 0.5 * dt3 * ad
          + 0.291667 * dt4 * ad2
          - 0.125 * dt5 * ad3
          + 0.0430556 * dt6 * ad4
        );
        Q(i + 12, i + 12) = qc * (
          dt
          - dt2 * ad
          + (2 / 3) * dt3 * ad2
          - (1 / 3) * dt4 * ad3
          + 0.133333 * dt5 * ad4
        );
      }
    }
    return Q;
  }

  /// calculate Phi
  template<int Dim>
  Eigen::Matrix<double, 3 * Dim, 3 * Dim> calcPhiSinger(
    const Eigen::Matrix<double, Dim, Dim> &Ad,
    double dt) {
    const Eigen::Matrix<double, Dim, Dim> I = Eigen::Matrix<double, Dim, Dim>::Identity();
    const Eigen::Matrix<double, Dim, Dim> Z = Eigen::Matrix<double, Dim, Dim>::Zero();
    const Eigen::Matrix<double, Dim, Dim> EXP = (-Ad * dt).exp();
    Eigen::Matrix<double, 3 * Dim, 3 * Dim> Phi = (Eigen::Matrix<double, 3 * Dim, 3 * Dim>() <<
                                                                                             I, dt * I,
      (Ad * dt - I + EXP) * Ad.inverse(),
      Z, I, (I - EXP) * Ad.inverse(),
      Z, Z, EXP).finished();
    return Phi;

  }

  /// calculate Lambda
  template<int Dim>
  Eigen::Matrix<double, 3 * Dim, 3 * Dim>
  calcLambdaSinger(const Eigen::Matrix<double, Dim, Dim> &Qc, const Eigen::Matrix<double, Dim, Dim> &Ad,
                   double delta_t, const double tau) {

    Eigen::Matrix<double, 3 * Dim, 3 * Dim> Lambda =
      calcPhiSinger<Dim>(Ad, tau) - calcQ3(Qc, tau) * (calcPhiSinger<Dim>(Ad, delta_t - tau).transpose())
                                    * calcQ_inv3(Qc, delta_t) * calcPhiSinger<Dim>(Ad, delta_t);

    ///Eigen::Matrix<double, 3*Dim, 3*Dim> Lambda = calcPhi3<Dim>(tau) - calcPsi3(Qc, delta_t, tau) * (calcPhi3<Dim>(delta_t));
    // Eigen::Matrix<double, 2*Dim, 2*Dim> Lambda = calcPhi<Dim>(tau) - calcQ(Qc, tau) * (calcPhi<Dim>(delta_t - tau).transpose())
    //                                                                         * calcQ_inv(Qc, delta_t) * calcPhi<Dim>(delta_t);

    return Lambda;
  }

  /// calculate Psi
  template<int Dim>
  Eigen::Matrix<double, 3 * Dim, 3 * Dim>
  calcPsiSinger(const Eigen::Matrix<double, Dim, Dim> &Qc, const Eigen::Matrix<double, Dim, Dim> &Ad,
                double delta_t, double tau) {

    Eigen::Matrix<double, 3 * Dim, 3 * Dim> Psi = calcQ3(Qc, tau) * (calcPhiSinger<Dim>(Ad, delta_t - tau).transpose())
                                                  * calcQ_inv3(Qc, delta_t);

    //Eigen::Matrix<double, 3*Dim, 3*Dim> Psi = calcQ3(Qc, tau) * (calcPhi3<Dim>(delta_t).transpose())
    //                                        * calcQ_inv3(Qc, tau);
    return Psi;
  }
}

#endif //ONLINE_FGO_GPUTILS_H
