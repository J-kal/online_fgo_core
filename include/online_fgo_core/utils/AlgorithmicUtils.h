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
// Created by haoming on 26.05.24.
//

#ifndef ONLINE_FGO_ALGORITHMICUTILS_H
#define ONLINE_FGO_ALGORITHMICUTILS_H
#pragma once

#include <iostream>
#include <vector>
#include <numeric>      // std::iota
#include <algorithm>    // std::sort, std::stable_sort

#include <gtsam/inference/Key.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>

namespace fgo::utils {
  inline std::tuple<gtsam::KeyVector, std::vector<size_t>> sortedKeyIndexes(const gtsam::KeyVector &v) {

    // initialize original index locations
    std::vector<size_t> idx(v.size());
    iota(idx.begin(), idx.end(), 0);

    // sort indexes based on comparing values in v
    // using std::stable_sort instead of std::sort
    // to avoid unnecessary index re-orderings
    // when v contains elements of equal values
    std::stable_sort(idx.begin(), idx.end(),
                     [&v](size_t i1, size_t i2) { return v[i1] < v[i2]; });

    gtsam::KeyVector orderedKeys;
    for (const auto &id: idx)
      orderedKeys.emplace_back(v[id]);

    return {orderedKeys, idx};
  }

  inline gtsam::Matrix rebuildJointMarginalMatrix(const gtsam::JointMarginal &m,
                                                  const gtsam::KeyVector &mKeyVector,
                                                  const std::vector<size_t> &mKeyVectorIndices) {
    std::map<std::pair<size_t, size_t>, gtsam::Matrix> index_block_map;
    const auto &m_matrix = m.fullMatrix();
    gtsam::Matrix m_rebuilt = gtsam::Matrix::Zero(m_matrix.rows(), m_matrix.cols());

    for (size_t i = 0; i < mKeyVector.size(); i++) {
      const auto index_i = mKeyVectorIndices[i];
      for (size_t j = 0; j < mKeyVector.size(); j++) {
        const auto index_j = mKeyVectorIndices[j];
        auto m_block = m.at(mKeyVector.at(i), mKeyVector.at(j));
        index_block_map.insert(std::make_pair(std::make_pair(index_i, index_j), m_block));
      }
    }

    size_t row_cursor = 0, col_cursor = 0;

    for (size_t i = 0; i < mKeyVector.size(); i++) {
      size_t row_tmp = 0;
      for (size_t j = 0; j < mKeyVector.size(); j++) {
        const auto &m_block = index_block_map.at(std::make_pair(i, j));
        m_rebuilt.block(row_cursor, col_cursor, m_block.rows(), m_block.cols()) = m_block;
        col_cursor += m_block.cols();
        row_tmp = m_block.rows();
      }
      row_cursor += row_tmp;
      col_cursor = 0;
    }


    return m_rebuilt;
  }


}
#endif //ONLINE_FGO_ALGORITHMICUTILS_H
