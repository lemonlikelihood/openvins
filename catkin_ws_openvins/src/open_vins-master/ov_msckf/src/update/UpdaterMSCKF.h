/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef OV_MSCKF_UPDATER_MSCKF_H
#define OV_MSCKF_UPDATER_MSCKF_H


#include <Eigen/Eigen>
#include "state/State.h"
#include "state/StateHelper.h"
#include "feat/Feature.h"
#include "feat/FeatureRepresentation.h"
#include "feat/FeatureInitializer.h"
#include "feat/FeatureInitializerOptions.h"
#include "utils/quat_ops.h"

#include "UpdaterHelper.h"
#include "UpdaterOptions.h"

#include <ros/ros.h>
#include <boost/math/distributions/chi_squared.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>


namespace ov_msckf {



    /**
     * @brief Will compute the system for our sparse features and update the filter.
     *
     * This class is responsible for computing the entire linear system for all features that are going to be used in an update.
     * This follows the original MSCKF, where we first triangulate features, we then nullspace project the feature Jacobian.
     * After this we compress all the measurements to have an efficient update and update the state.
     */
    class UpdaterMSCKF {

    public:


        /**
         * @brief Default constructor for our MSCKF updater
         *
         * Our updater has a feature initializer which we use to initialize features as needed.
         * Also the options allow for one to tune the different parameters for update.
         *
         * @param options Updater options (include measurement noise value)
         * @param feat_init_options Feature initializer options
         */

        // 初始化更新参数和卡方验验参数，卡方验验方差和特征点初始化信息
        UpdaterMSCKF(UpdaterOptions &options, FeatureInitializerOptions &feat_init_options) : _options(options){

            // Save our raw pixel noise squared
            _options.sigma_pix_sq = std::pow(_options.sigma_pix,2);

            // Save our feature initializer 利用特征点初始化选项来初始化特征点初始化器
            initializer_feat = new FeatureInitializer(feat_init_options);

            // Initialize the chi squared test table with confidence level 0.95
            // https://github.com/KumarRobotics/msckf_vio/blob/050c50defa5a7fd9a04c1eed5687b405f02919b5/src/msckf_vio.cpp#L215-L221
            for (int i = 1; i < 500; i++) {                        // 初始化卡方校验表
                boost::math::chi_squared chi_squared_dist(i);
                chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
            }

        }


        /**
         * @brief Given tracked features, this will try to use them to update the state.
         *
         * @param state State of the filter
         * @param feature_vec Features that can be used for update
         */
        void update(State *state, std::vector<Feature*>& feature_vec);



    protected:


        /// Options used during update
        UpdaterOptions _options;                         // msckf 更新参数，包括像素方差和卡方校验的方差，用来建建校校验验表

        /// Feature initializer class object
        FeatureInitializer* initializer_feat;           // 3D 特征点初始化然然参数，包括最远最近距离等

        /// Chi squared 95th percentile table (lookup would be size of residual)
        std::map<int, double> chi_squared_table;       // 卡方校验表


    };




}




#endif //OV_MSCKF_UPDATER_MSCKF_H


