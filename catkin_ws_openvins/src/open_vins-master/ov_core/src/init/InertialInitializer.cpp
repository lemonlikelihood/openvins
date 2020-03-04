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
#include "InertialInitializer.h"


using namespace ov_core;

// 存储接收到的imu数据，增加新的，删除旧的
void InertialInitializer::feed_imu(double timestamp, Eigen::Matrix<double,3,1> wm, Eigen::Matrix<double,3,1> am) {

    // Create our imu data object
    IMUDATA data;
    data.timestamp = timestamp;
    data.wm = wm;
    data.am = am;

    // Append it to our vector
    imu_data.emplace_back(data);

    // Delete all measurements older than three of our initialization windows   // 保证窗口大小为 3 * _window_length , 剔除老的imu数据
    auto it0 = imu_data.begin();
    while(it0 != imu_data.end() && it0->timestamp < timestamp-3*_window_length) {
        it0 = imu_data.erase(it0);
    }

}





bool InertialInitializer::initialize_with_imu(double &time0, Eigen::Matrix<double,4,1> &q_GtoI0, Eigen::Matrix<double,3,1> &b_w0,
                                              Eigen::Matrix<double,3,1> &v_I0inG, Eigen::Matrix<double,3,1> &b_a0, Eigen::Matrix<double,3,1> &p_I0inG) {

    // Return if we don't have any measurements
    if(imu_data.empty()) {
        return false;
    }

    // Newest imu timestamp
    double newesttime = imu_data.at(imu_data.size()-1).timestamp;

    // First lets collect a window of IMU readings from the newest measurement to the oldest     // 1. 将imu_data根据先后顺序分成两组IMU数据存储到数组中
    std::vector<IMUDATA> window_newest, window_secondnew;
    for(IMUDATA data : imu_data) {
        if(data.timestamp > newesttime-1*_window_length && data.timestamp <= newesttime-0*_window_length) {
            window_newest.push_back(data);                                                       //最新，前0.75s 内的数据
        }
        if(data.timestamp > newesttime-2*_window_length && data.timestamp <= newesttime-1*_window_length) {
            window_secondnew.push_back(data);                                                    //次新， 前1.5s 到 0.75s 的数据
        }
    }

    // Return if both of these failed
    if(window_newest.empty() || window_secondnew.empty()) {
        //ROS_WARN("InertialInitializer::initialize_with_imu(): unable to select window of IMU readings, not enough readings");
        return false;
    }

    // Calculate the sample variance for the newest one
    Eigen::Matrix<double,3,1> a_avg = Eigen::Matrix<double,3,1>::Zero();
    for(IMUDATA data : window_newest) {
        a_avg += data.am;
    }
    a_avg /= (int)window_newest.size();
    double a_var = 0;
    for(IMUDATA data : window_newest) {
        a_var += (data.am-a_avg).dot(data.am-a_avg);
    }
    a_var = std::sqrt(a_var/((int)window_newest.size()-1));

    // If it is below the threshold just return   // 2. 先统计最新的状态是否有变化（避免出现静止，或者匀速运动），若加速度的变化太小则返回初始化失败
    if(a_var < _imu_excite_threshold) {
        ROS_WARN("InertialInitializer::initialize_with_imu(): no IMU excitation, below threshold %.4f < %.4f",a_var,_imu_excite_threshold);
        return false;
    }

    // Return if we don't have any measurements
    //if(imu_data.size() < 200) {
    //    return false;
    //}

    // Sum up our current accelerations and velocities     // 3. 若上述条件通过，则统计次新组数据的平均值，并没有检查当前的协方差是否静止？？？
    Eigen::Vector3d linsum = Eigen::Vector3d::Zero();
    Eigen::Vector3d angsum = Eigen::Vector3d::Zero();
    for(size_t i=0; i<window_secondnew.size(); i++) {
        linsum += window_secondnew.at(i).am;
        angsum += window_secondnew.at(i).wm;
    }

    // Calculate the mean of the linear acceleration and angular velocity
    Eigen::Vector3d linavg = Eigen::Vector3d::Zero();
    Eigen::Vector3d angavg = Eigen::Vector3d::Zero();
    linavg = linsum/window_secondnew.size();
    angavg = angsum/window_secondnew.size();

    // Get z axis, which alines with -g (z_in_G=0,0,1)          // 构建imu 坐标系下的xyz轴
    Eigen::Vector3d z_axis = linavg/linavg.norm();

    // Create an x_axis
    Eigen::Vector3d e_1(1,0,0);

    // Make x_axis perpendicular to z
    Eigen::Vector3d x_axis = e_1-z_axis*z_axis.transpose()*e_1;
    x_axis= x_axis/x_axis.norm();

    // Get z from the cross product of these two
    Eigen::Matrix<double,3,1> y_axis = skew_x(z_axis)*x_axis;

    // From these axes get rotation
    Eigen::Matrix<double,3,3> Ro;                          // 4. 计算世界坐标系下 g 到IMU 坐标系下 g 的旋转 q_GtoI
    Ro.block(0,0,3,1) = x_axis;
    Ro.block(0,1,3,1) = y_axis;
    Ro.block(0,2,3,1) = z_axis;

    // Create our state variables
    Eigen::Matrix<double,4,1> q_GtoI = rot_2_quat(Ro);

    // Set our biases equal to our noise (subtract our gravity from accelerometer bias)
    Eigen::Matrix<double,3,1> bg = angavg;                // 5. 计算bg,ba 初始值,静止初始化
    Eigen::Matrix<double,3,1> ba = linavg - quat_2_Rot(q_GtoI)*_gravity;


    // Set our state variables
    time0 = window_secondnew.at(window_secondnew.size()-1).timestamp;
    q_GtoI0 = q_GtoI;
    b_w0 = bg;
    v_I0inG = Eigen::Matrix<double,3,1>::Zero();
    b_a0 = ba;
    p_I0inG = Eigen::Matrix<double,3,1>::Zero();

    // Done!!!
    return true;


}




