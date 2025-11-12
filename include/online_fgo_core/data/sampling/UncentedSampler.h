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
// Created by haoming on 22.05.24.
//

#ifndef ONLINE_FGO_UNCENTEDSAMPLER_H
#define ONLINE_FGO_UNCENTEDSAMPLER_H

#include <memory>
#include <NumCpp.hpp>
#include <gtsam/linear/Sampler.h>
#include <gtsam/base/Value.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose3.h>

namespace fgo::data::sampler {

  class SigmaPoints {
  public:
    typedef std::shared_ptr<SigmaPoints> SharedPtr;

    size_t num_sigmas() const { return n_sigma_points_; }

    virtual gtsam::Matrix sigma_points(const gtsam::Vector &mean,
                                       const gtsam::Matrix &cov,
                                       bool isTrangularized = false) = 0;

  protected:
    size_t n_state_;
    size_t n_sigma_points_;
    double lambda_;
    gtsam::Vector weights_;
    double kappa_;
    gtsam::Vector Wc_;
    gtsam::Vector Wm_;

  };

  class MerweScaledSigmaPoints final : public SigmaPoints {
  public:
    typedef std::shared_ptr<MerweScaledSigmaPoints> SharedPtr;

    explicit MerweScaledSigmaPoints(size_t n, double alpha = 1e-3, double beta = 2., double kappa = 0.);

    gtsam::Matrix sigma_points(const gtsam::Vector &mean,
                               const gtsam::Matrix &cov,
                               bool isTrangularized = false) override;

  private:
    double alpha_;
    double beta_;

  };

  class JulierSigmaPoints final : public SigmaPoints {
  public:
    typedef std::shared_ptr<JulierSigmaPoints> SharedPtr;

    explicit JulierSigmaPoints(size_t n, double kappa);

    gtsam::Matrix sigma_points(const gtsam::Vector &mean,
                               const gtsam::Matrix &cov,
                               bool isTrangularized = false) override;

    ~JulierSigmaPoints() = default;
  };

  class SimplexSigmaPoints final : public SigmaPoints {
  public:
    typedef std::shared_ptr<SimplexSigmaPoints> SharedPtr;

    explicit SimplexSigmaPoints(size_t n, double alpha = 1.);

    gtsam::Matrix sigma_points(const gtsam::Vector &mean,
                               const gtsam::Matrix &cov,
                               bool isTrangularized = false) override;

    ~SimplexSigmaPoints() = default;

  private:
    double alpha_;

  };


  struct SamplerConfig {
    typedef std::shared_ptr<SamplerConfig> SharedPtr;
    std::string sigmaType = "Merwe";
    double alpha = 1e-3;
    double beta = 2.;
    double kappa = 0.;
  };

  class UnscentedSampler final : public gtsam::Sampler {
  public:
    typedef std::shared_ptr<UnscentedSampler> SharedPtr;

    explicit UnscentedSampler(const SamplerConfig::SharedPtr &param,
                              const gtsam::Vector &mean,
                              const gtsam::noiseModel::Base::shared_ptr &model);

    explicit UnscentedSampler(const SamplerConfig::SharedPtr &param,
                              const gtsam::Vector &mean,
                              const gtsam::Matrix &cov);

    explicit UnscentedSampler(const SamplerConfig::SharedPtr &param, const gtsam::Vector &sigmas);


    //void unscented_transform(std::function<gtsam::Vector(const gtsam::Values& x,
    //                                                     boost::optional<std::vector<gtsam::Matrix>&> H)>const& func);
    //void unscented_transform(std::function<gtsam::Vector(const gtsam::Values& x)> const& func);

    gtsam::Matrix samples() const;

    ~UnscentedSampler() = default;

  private:
    SigmaPoints::SharedPtr sigma_generator_;
    SamplerConfig::SharedPtr param_;
    gtsam::Matrix cov_;
  };


}


#endif //ONLINE_FGO_UNCENTEDSAMPLER_H
