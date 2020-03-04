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
#ifndef OV_CORE_FEATURE_H
#define OV_CORE_FEATURE_H


#include <vector>
#include <iostream>
#include <unordered_map>
#include <Eigen/Eigen>

namespace ov_core {

    /**
     * @brief Sparse feature class used to collect measurements
     *
     * This feature class allows for holding of all tracking information for a given feature.
     * Each feature has a unique ID assigned to it, and should have a set of feature tracks alongside it.
     * See the FeatureDatabase class for details on how we load information into this, and how we delete features.
     */
    class Feature {

    public:

        /// Unique ID of this feature
        size_t featid;

        /// If this feature should be deleted
        bool to_delete;

        /// UV coordinates that this feature has been seen from (mapped by camera ID)
        /// 可能存在多个相机stereo，size_t 表示camera_ID 0 表示左相机，1表示右相机，uvs表示对应相机里能够观测到该特征点的图片序列的图像坐标系坐标，没有去畸变（原始坐标）
        std::unordered_map<size_t, std::vector<Eigen::VectorXf>> uvs;

        /// UV normalized coordinates that this feature has been seen from (mapped by camera ID)
        /// 可能存在多个相机stereo，size_t 表示camera_ID 0 表示左相机，1表示右相机，uvs_norm表示对应相机里能够观测到该特征点的图片序列归一化平面坐标，去畸变了（归一化平面）
        std::unordered_map<size_t, std::vector<Eigen::VectorXf>> uvs_norm;

        /// Timestamps of each UV measurement (mapped by camera ID)
        /// 可能存在多个相机stereo，size_t 表示camera_ID 0 表示左相机，1表示右相机，timestamps表示对应相机里能够观测到该特征点的图片序列时间戳
        std::unordered_map<size_t, std::vector<double>> timestamps;

        /// What camera ID our pose is anchored in!! By default the first measurement is the anchor.
        int anchor_cam_id = -1;

        /// Timestamp of anchor clone
        double anchor_clone_timestamp;

        /// Triangulated position of this feature, in the anchor frame
        Eigen::Vector3d p_FinA;

        /// Triangulated position of this feature, in the global frame
        Eigen::Vector3d p_FinG;


        /**
         * @brief Remove measurements that do not occur at passed timestamps
         *
         * Given a series of valid timestamps, this will remove all measurements that have not occurred at these times.
         * This would normally be used to ensure that the measurements that we have occur at our clone times.
         *
         * @param valid_times Vector of timestamps that our measurements must occur at
         */
        void clean_old_measurements(std::vector<double> valid_times);

    };

}


#endif /* OV_CORE_FEATURE_H */