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
#ifndef OV_MSCKF_STATE_H
#define OV_MSCKF_STATE_H


#include <vector>
#include <unordered_map>

#include "types/Type.h"
#include "types/IMU.h"
#include "types/Vec.h"
#include "types/PoseJPL.h"
#include "types/Landmark.h"
#include "StateOptions.h"

using namespace ov_core;


namespace ov_msckf {

    /**
     * @brief State of our filter
     *
     * This state has all the current estimates for the filter.
     * This system is modeled after the MSCKF filter, thus we have a sliding window of clones.
     * We additionally have more parameters for online estimation of calibration and SLAM features.
     * We also have the covariance of the system, which should be managed using the StateHelper class.
     */
    class State {

    public:

        /**
         * @brief Default Constructor (will initialize variables to defaults)
         * @param options_ Options structure containing filter options
         */
        State(StateOptions &options_) : _options(options_) {             // 利用StateOptions 来构造初始的状态向量
            initialize_variables();
        }

        ~State() {}

        /**
         * @brief Initializes pointers and covariance
         *
         * TODO: Read initial values and covariance from options
         */
        void initialize_variables();                                    // 初始化状态向量和协方差


        /**
         * @brief Given an update vector for the **entire covariance**, updates each variable
         * @param dx Correction vector for the entire filter state
         */
        void update(const Eigen::MatrixXd dx) {
            for (size_t i = 0; i < _variables.size(); i++) {
                _variables[i]->update(dx.block(_variables[i]->id(), 0, _variables[i]->size(), 1));
            }
        }

        /**
         * @brief Set the current timestamp of the filter
         * @param timestamp New timestamp
         */
        void set_timestamp(double timestamp) {                          // 设置当前状态向量的时间戳
            _timestamp = timestamp;
        }

        /**
         * @brief Will return the timestep that we will marginalize next.
         *
         * As of right now, since we are using a sliding window, this is the oldest clone.
         * But if you wanted to do a keyframe system, you could selectively marginalize clones.
         *
         * @return timestep of clone we will marginalize
         */
        double margtimestep() {                                      // 返回clones_imu里面最早的clone时间戳，也就是要margin掉最早的Image
            double time = INFINITY;
            for (std::pair<const double, PoseJPL *> &clone_imu : _clones_IMU) {
                if (clone_imu.first < time) {
                    time = clone_imu.first;
                }
            }
            return time;
        }

        /// Access current timestamp
        double timestamp() {                                              // 返回当前状态向量的时间戳
            return _timestamp;
        }

        /// Access options struct
        StateOptions& options() {                                         // 返回当前状态向里的参数
            return _options;
        }

        /// Access to IMU type
        IMU* imu() {                                                      // 返回IMU状态
            return _imu;
        }

        /// Access covariance matrix
        Eigen::MatrixXd &Cov() {                                          // 返回协方差矩阵
            return _Cov;
        }

        /// Get size of covariance
        size_t n_vars() {                                                  // 返回所有状态的协方差矩阵的维度，当前状态向量中包含的所有参数的维度
            return _Cov.rows();
        }

        /// Access imu-camera time offset type
        Vec* calib_dt_CAMtoIMU() {                                         // 返回td估计值
            return _calib_dt_CAMtoIMU;
        }

        /// Access to a given clone
        PoseJPL* get_clone(double timestamp) {                              // 返回某个clone的位姿
            return _clones_IMU.at(timestamp);
        }

        /// Access to all current clones in the state
        std::unordered_map<double, PoseJPL*> get_clones() {                 // 返回所有clone的位姿
            return _clones_IMU;
        }

        /// Get current number of clones
        size_t n_clones() {                                                 // 返回当前clone的大小
            return _clones_IMU.size();
        }

        /// Access to a given camera extrinsics
        PoseJPL* get_calib_IMUtoCAM(size_t id) {                            // 返回某一个相机的外参数
            assert((int)id<_options.num_cameras);
            return _calib_IMUtoCAM.at(id);
        }

        /// Access to all current extrinsic calibration between IMU and each camera
        std::unordered_map<size_t, PoseJPL*> get_calib_IMUtoCAMs() {        // 返回所有相机的外参数
            return _calib_IMUtoCAM;
        }

        /// Access to a given camera intrinsics
        Vec* get_intrinsics_CAM(size_t id) {                                 // 返回某一个相机的内参数
            assert((int)id<_options.num_cameras);
            return _cam_intrinsics.at(id);
        }

        /// Access to a given camera intrinsics
        bool& get_model_CAM(size_t id) {                                      // 返回某一个相机的畸变模型
            assert((int)id<_options.num_cameras);
            return _cam_intrinsics_model.at(id);
        }

        /// Add a SLAM feature
        void insert_SLAM_feature(size_t featid, Landmark* landmark){           // 插入一个新的slam feature,key 是 featid
            _features_SLAM.insert({featid, landmark});
        }

        /// Access SLAM features
        std::unordered_map<size_t, Landmark*> &features_SLAM(){                // 返回所有的slam feature
            return _features_SLAM;
        }


    protected:

        /// Current timestamp (should be the last update time!)
        double _timestamp;                                            // 当前状态向量的时间戳

        /// Struct containing filter options
        StateOptions _options;                                        // 状态向量的参数

        /// Pointer to the "active" IMU state (q_GtoI, p_IinG, v_IinG, bg, ba)
        IMU *_imu;                                                    // imu状态

        /// Map between imaging times and clone poses (q_GtoIi, p_IiinG)
        std::unordered_map<double, PoseJPL*> _clones_IMU;             // 滑动窗口里每一个Image的 IMU poses, double 时间戳，图片对应的时间戳

        /// Our current set of SLAM features (3d positions)
        std::unordered_map<size_t, Landmark*> _features_SLAM;         // 当前的slam_feature , size_t key 是 featID，自由度为3

        /// Time offset base IMU to camera (t_imu = t_cam + t_off)
        Vec *_calib_dt_CAMtoIMU;                                      // 当前滑动窗口里的每一个Image 的时间同步误差

        /// Calibration poses for each camera (R_ItoC, p_IinC)
        std::unordered_map<size_t, PoseJPL*> _calib_IMUtoCAM;         // 每一个相机的外参

        /// Camera intrinsics
        std::unordered_map<size_t, Vec*> _cam_intrinsics;             // 每一个相机的内参

        /// What distortion model we are using (false=radtan, true=fisheye)
        std::unordered_map<size_t, bool> _cam_intrinsics_model;       // 每一个相机的畸变模型

        /// Covariance of all active variables
        Eigen::MatrixXd _Cov;                                         // 整个状态向量的协方差

        /// Vector of variables
        std::vector<Type *> _variables;                               // 所有状态量的指针


    private:

        // Define that the state helper is a friend class of this class
        // This will allow it to access the below functions which should normally not be called
        // This prevents a developer from thinking that the "insert clone" will actually correctly add it to the covariance
        friend class StateHelper;

        /// Insert a new clone specified by its timestep and type           // 往滑动窗口里加入新的clone(image)
        void insert_clone(double timestamp, PoseJPL *pose) {
            _clones_IMU.insert({timestamp, pose});
        }

        /// Removes a clone from a specific timestep
        void erase_clone(double timestamp) {                               // 从滑动窗口里删除一个clone(image)
            _clones_IMU.erase(timestamp);
        }

        /// Access all current variables in our covariance
        std::vector<Type *> &variables() {                                 // 返回所有的状态量
            return _variables;
        }

        /// Access a specific variable by its id
        Type *variables(size_t i) {                                       // 返回某一个状态量
            return _variables.at(i);
        }

        /// Insert a new variable into our vector (                       // 往状态向量里加入一个新的状态
        void insert_variable(Type *newType) {
            _variables.push_back(newType);
        }


    };

}

#endif //OV_MSCKF_STATE_H