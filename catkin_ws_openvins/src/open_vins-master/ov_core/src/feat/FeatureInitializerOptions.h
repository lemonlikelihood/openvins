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
#ifndef OV_CORE_INITIALIZEROPTIONS_H
#define OV_CORE_INITIALIZEROPTIONS_H


namespace ov_core {


    /**
     * @brief Struct which stores all our feature initializer options
     */
     // 特征点初始化选项
    struct FeatureInitializerOptions {

        /// Max runs for Gauss Newton
        int max_runs = 20;                      // 三角化时高斯牛顿迭代的最大次数

        /// Init lambda for LM optimization
        double init_lamda = 1e-3;               // LM 算法初始lamda的值

        /// Max lambda for LM optimization
        double max_lamda = 1e10;                // LM 算法lamda的值阈值，大于此阈值算法直接退出

        /// Cutoff for dx increment to consider as converged
        double min_dx = 1e-6;                   // dx < min_dx 表示收敛，dx已经小于阈值了

        /// Cutoff for cost decrement to consider as converged
        double min_dcost = 1e-6;               // 两次的cost差相对cost本身比例已经很小了

        /// Multiplier to increase/decrease lambda
        double lam_mult = 10;                  // LM 算法中增加较小lamda的比例

        /// Minimum distance to accept triangulated features
        double min_dist = 0.25;                // 三角化解出3D点在当前帧坐标系中的坐标，z最近距离0.25m，小于0.25m认为三角化结果不准确

        /// Maximum distance to accept triangulated features
        double max_dist = 40;                  // 三角化解出3D点在当前帧坐标系中的坐标，z最远距离40m，大于40m认为三角化结果不准确

        /// Max baseline ratio to accept triangulated features
        double max_baseline = 40;              // 三角化可以接受的最大基线

        /// Max condition number of linear triangulation matrix accept triangulated features
        double max_cond_number = 1000;        // SVD分解时的条件数，判断最大的特征值和最小的特征值之间的比率，来决定求解是否病态


    };

}

#endif //OV_CORE_INITIALIZEROPTIONS_H