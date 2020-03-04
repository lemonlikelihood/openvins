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
#include "Propagator.h"



using namespace ov_core;
using namespace ov_msckf;



// 传入当前状态向量和当前Image时间戳,当接收到图片信息才会调用这个函数，此时状态向量的时间戳小于当前clone的时间戳
void Propagator::propagate_and_clone(State* state, double timestamp) {

    // If the difference between the current update time and state is zero
    // We should crash, as this means we would have two clones at the same time!!!!
    if(state->timestamp() == timestamp) {
        std::cerr << "Propagator::propagate_and_clone(): Propagation called again at same timestep at last update timestep!!!!" << std::endl;
        std::cerr << "Propagator::propagate_and_clone(): " << state->timestamp() << " vs " << timestamp << " timestamps" << std::endl;
        std::cerr << __FILE__ << " on line " << __LINE__ << std::endl;
        std::exit(EXIT_FAILURE);
    }

    // We should crash if we are trying to propagate backwards
    if(state->timestamp() > timestamp) {
        std::cerr << "Propagator::propagate_and_clone(): Propagation called trying to propagate backwards in time!!!!" << std::endl;
        std::cerr << "Propagator::propagate_and_clone(): desired propagation = " << (timestamp-state->timestamp()) << std::endl;
        std::cerr << __FILE__ << " on line " << __LINE__ << std::endl;
        std::exit(EXIT_FAILURE);
    }

    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Set the last time offset value if we have just started the system up
    if(last_prop_time_offset == -INFINITY) {
        last_prop_time_offset = state->calib_dt_CAMtoIMU()->value()(0);                  // 估计出来的dt
    }

    // Get what our IMU-camera offset should be (t_imu = t_cam + calib_dt)
    double t_off_new = state->calib_dt_CAMtoIMU()->value()(0);

    // First lets construct an IMU vector of measurements we need
    double time0 = state->timestamp()+last_prop_time_offset;
    double time1 = timestamp+t_off_new;                                     // 当前Image所对应的IMU时间戳
    vector<IMUDATA> prop_data = Propagator::select_imu_readings(imu_data,time0,time1);   //获取IMU数据 time0 到 time1的插值

    // We are going to sum up all the state transition matrices, so we can do a single large multiplication at the end
    // Phi_summed = Phi_i*Phi_summed
    // Q_summed = Phi_i*Q_summed*Phi_i^T + Q_i
    // After summing we can multiple the total phi to get the updated covariance
    // We will then add the noise to the IMU portion of the state
    Eigen::Matrix<double,15,15> Phi_summed = Eigen::Matrix<double,15,15>::Identity();
    Eigen::Matrix<double,15,15> Qd_summed = Eigen::Matrix<double,15,15>::Zero();
    double dt_summed = 0;

    // Loop through all IMU messages, and use them to move the state forward in time
    // This uses the zero'th order quat, and then constant acceleration discrete
    for(size_t i=0; i<prop_data.size()-1; i++) {

        // Get the next state Jacobian and noise Jacobian for this IMU reading
        Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Zero();
        Eigen::Matrix<double, 15, 15> Qdi = Eigen::Matrix<double, 15, 15>::Zero();
        predict_and_compute(state, prop_data.at(i), prop_data.at(i+1), F, Qdi);      // 传播当前的IMU状态(IMU状态向前传播，用当前接收到的IMU a以及w来计算当前的状态)并计算F和Qd

        // Next we should propagate our IMU covariance
        // Pii' = F*Pii*F.transpose() + G*Q*G.transpose()
        // Pci' = F*Pci and Pic' = Pic*F.transpose()
        // NOTE: Here we are summing the state transition F so we can do a single mutiplication later
        // NOTE: Phi_summed = Phi_i*Phi_summed
        // NOTE: Q_summed = Phi_i*Q_summed*Phi_i^T + G*Q_i*G^T
        Phi_summed = F * Phi_summed;
        Qd_summed = F * Qd_summed * F.transpose() + Qdi;
        Qd_summed = 0.5*(Qd_summed+Qd_summed.transpose());
        dt_summed +=  prop_data.at(i+1).timestamp-prop_data.at(i).timestamp;
    }

    // Last angular velocity (used for cloning when estimating time offset)
    Eigen::Matrix<double,3,1> last_w = prop_data.at(prop_data.size()-2).wm - state->imu()->bias_g();

    // For now assert that our IMU is at the top left of the covariance
    assert(state->imu()->id()==0);

    // Do the update to the covariance with our "summed" state transition and IMU noise addition...
    auto &Cov = state->Cov();
    size_t imu_id = state->imu()->id();
    Cov.block(imu_id,0,15,state->n_vars()) = Phi_summed*Cov.block(imu_id,0,15,state->n_vars());                   // n_vars返回协方差的维度
    Cov.block(0,imu_id,state->n_vars(),15) = Cov.block(0,imu_id,state->n_vars(),15)*Phi_summed.transpose();       // 协方差的更新
    Cov.block(imu_id,imu_id,15,15) += Qd_summed;

    // Ensure the covariance is symmetric
    Cov = 0.5*(Cov+Cov.transpose());                                                        // 保证协方差是对称矩阵

    // Set timestamp data
    state->set_timestamp(timestamp);                                                       // 更新状态向量的timestamp为当前已传播clone的时间戳
    last_prop_time_offset = t_off_new;                                                     // 设置当前时间戳的偏移

    // Now perform stochastic cloning
    StateHelper::augment_clone(state, last_w);                                             // 传播完成进行状态增广

}


std::vector<Propagator::IMUDATA> Propagator::select_imu_readings(const std::vector<IMUDATA>& imu_data, double time0, double time1) {

    // Our vector imu readings
    std::vector<Propagator::IMUDATA> prop_data;

    // Ensure we have some measurements in the first place!
    if(imu_data.empty()) {
        std::cerr << "Propagator::select_imu_readings(): There are no IMU measurements!!!!!" << std::endl;
        std::cerr << "Propagator::select_imu_readings(): IMU-CAMERA are likely messed up, check time offset value!!!" << std::endl;
        std::cerr << __FILE__ << " on line " << __LINE__ << std::endl;
        return prop_data;
    }

    // Sort our imu data (handles any out of order measurements)
    //std::sort(imu_data.begin(), imu_data.end(), [](const IMUDATA i, const IMUDATA j){
    //    return i.timestamp < j.timestamp;
    //});

    // Loop through and find all the needed measurements to propagate with
    // Note we split measurements based on the given state time, and the update timestamp
    for(size_t i=0; i<imu_data.size()-1; i++) {

        // START OF THE INTEGRATION PERIOD
        // If the next timestamp is greater then our current state time
        // And the current is not greater then it yet...
        // Then we should "split" our current IMU measurement
        // 1. 在起始点时间戳time0 正好位于两帧imu读数之间，则用插值得到time0的虚拟读数
        if(imu_data.at(i+1).timestamp > time0 && imu_data.at(i).timestamp < time0) {
            IMUDATA data = Propagator::interpolate_data(imu_data.at(i),imu_data.at(i+1), time0);
            prop_data.push_back(data);
            //ROS_INFO("propagation #%d = CASE 1 = %.3f => %.3f", (int)i,data.timestamp-prop_data.at(0).timestamp,time0-prop_data.at(0).timestamp);
            continue;
        }

        // MIDDLE OF INTEGRATION PERIOD
        // If our imu measurement is right in the middle of our propagation period
        // Then we should just append the whole measurement time to our propagation vector
        // 2. 正好处于time0到time1之间的imu读数，直接存储
        if(imu_data.at(i).timestamp >= time0 && imu_data.at(i+1).timestamp <= time1) {
            prop_data.push_back(imu_data.at(i));
            //ROS_INFO("propagation #%d = CASE 2 = %.3f",(int)i,imu_data.at(i).timestamp-prop_data.at(0).timestamp);
            continue;
        }

        // END OF THE INTEGRATION PERIOD
        // If the current timestamp is greater then our update time
        // We should just "split" the NEXT IMU measurement to the update time,
        // NOTE: we add the current time, and then the time at the end of the interval (so we can get a dt)
        // NOTE: we also break out of this loop, as this is the last IMU measurement we need!

        // case 3-1 结尾的地方比较复杂，如果i+1 和i 都大于 time1 则用i-1和i 来插值出time1 的读数
        if(imu_data.at(i+1).timestamp > time1) {
            // If we have a very low frequency IMU then, we could have only recorded the first integration (i.e. case 1) and nothing else
            // In this case, both the current IMU measurement and the next is greater than the desired intepolation, thus we should just cut the current at the desired time
            // Else, we have hit CASE2 and this IMU measurement is not past the desired propagation time, thus add the whole IMU reading
            if(imu_data.at(i).timestamp > time1) {
                IMUDATA data = interpolate_data(imu_data.at(i-1), imu_data.at(i), time1);
                prop_data.push_back(data);
                //ROS_INFO("propagation #%d = CASE 3.1 = %.3f => %.3f", (int)i,imu_data.at(i).timestamp-prop_data.at(0).timestamp,imu_data.at(i).timestamp-time0);
            } else {      // case 3-2 如果i+1 大于 time1，而i 小于time1，,则存储i 的读数
                prop_data.push_back(imu_data.at(i));
                //ROS_INFO("propagation #%d = CASE 3.2 = %.3f => %.3f", (int)i,imu_data.at(i).timestamp-prop_data.at(0).timestamp,imu_data.at(i).timestamp-time0);
            }
            // If the added IMU message doesn't end exactly at the camera time
            // Then we need to add another one that is right at the ending time
            //case 3-3 如果结尾time1 没有数据，则用i 和i+1来插值虚拟的time1 时刻imu读数
            if(prop_data.at(prop_data.size()-1).timestamp != time1) {
                IMUDATA data = interpolate_data(imu_data.at(i), imu_data.at(i+1), time1);
                prop_data.push_back(data);
                //ROS_INFO("propagation #%d = CASE 3.3 = %.3f => %.3f", (int)i,data.timestamp-prop_data.at(0).timestamp,data.timestamp-time0);
            }
            break;
        }

    }

    // Check that we have at least one measurement to propagate with
    // 判断prop_data 个数是否为零
    if(prop_data.empty()) {
        std::cerr << "Propagator::select_imu_readings(): There are not enough measurements to propagate with " << (int)prop_data.size() << " of 2" << std::endl;
        std::cerr << "Propagator::select_imu_readings(): IMU-CAMERA time offset is likely messed up, check time offset value!!!" << std::endl;
        std::cerr << __FILE__ << " on line " << __LINE__ << std::endl;
        return prop_data;
    }

    // If we did not reach the whole integration period (i.e., the last inertial measurement we have is smaller then the time we want to reach)
    // Then we should just "stretch" the last measurement to be the whole period
    // 判断，imu数据是否未达到time1时刻
    if(imu_data.at(imu_data.size()-1).timestamp <= time1) {
        std::cerr << "Propagator::select_imu_readings(): There are not enough measurements to propagate with " << (time1-imu_data.at(imu_data.size()-1).timestamp) << " sec missing" << std::endl;
        std::cerr << "Propagator::select_imu_readings(): IMU-CAMERA time offset is likely messed up, check time offset value!!!" << std::endl;
        std::cerr << __FILE__ << " on line " << __LINE__ << std::endl;
        return prop_data;
    }

    // Loop through and ensure we do not have an zero dt values
    // This would cause the noise covariance to be Infinity
    // 若imu 数据中存在时间间隔过小的 1e-12，则去除，避免引起协方差无穷大
    for (size_t i=0; i < prop_data.size()-1; i++){
        if (std::abs(prop_data.at(i+1).timestamp-prop_data.at(i).timestamp) < 1e-12){
            std::cerr << "Propagator::select_imu_readings(): Zero DT between " << i << " and " << i+1 << " measurements (dt = " << (prop_data.at(i+1).timestamp-prop_data.at(i).timestamp) << ")" << std::endl;
            prop_data.erase(prop_data.begin()+i);
            i--;
        }
    }

    // Check that we have at least one measurement to propagate with
    // 若IMU数据少于2，则返回
    if(prop_data.size() < 2) {
        std::cerr << "Propagator::select_imu_readings(): There are not enough measurements to propagate with " << (int)prop_data.size() << " of 2" << std::endl;
        std::cerr << "Propagator::select_imu_readings(): IMU-CAMERA time offset is likely messed up, check time offset value!!!" << std::endl;
        std::cerr << __FILE__ << " on line " << __LINE__ << std::endl;
        return prop_data;
    }

    // Success :D
    return prop_data;

}

// 传播新的状态，计算F和Qd
void Propagator::predict_and_compute(State *state, const IMUDATA data_minus, const IMUDATA data_plus,
                                     Eigen::Matrix<double,15,15> &F, Eigen::Matrix<double,15,15> &Qd) {

    // Set them to zero
    F.setZero();
    Qd.setZero();

    // Time elapsed over interval
    double dt = data_plus.timestamp-data_minus.timestamp;
    assert(data_plus.timestamp>data_minus.timestamp);

    // Corrected imu measurements
    Eigen::Matrix<double,3,1> w_hat = data_minus.wm - state->imu()->bias_g();
    Eigen::Matrix<double,3,1> a_hat = data_minus.am - state->imu()->bias_a();
    Eigen::Matrix<double,3,1> w_hat2 = data_plus.wm - state->imu()->bias_g();
    Eigen::Matrix<double,3,1> a_hat2 = data_plus.am - state->imu()->bias_a();

    // Compute the new state mean value
    Eigen::Vector4d new_q;
    Eigen::Vector3d new_v, new_p;
    if(state->options().use_rk4_integration) predict_mean_rk4(state, dt, w_hat, a_hat, w_hat2, a_hat2, new_q, new_v, new_p);  // 使用rk4来预测pvq的均值，这一步相当于求解观测值
    else predict_mean_discrete(state, dt, w_hat, a_hat, w_hat2, a_hat2, new_q, new_v, new_p);

    // Get the locations of each entry of the imu state          求解误差状态转移矩阵F和G，这一步相当于求解估计值
    int th_id = state->imu()->q()->id()-state->imu()->id();
    int p_id = state->imu()->p()->id()-state->imu()->id();
    int v_id = state->imu()->v()->id()-state->imu()->id();
    int bg_id = state->imu()->bg()->id()-state->imu()->id();
    int ba_id = state->imu()->ba()->id()-state->imu()->id();

    // Allocate noise Jacobian
    Eigen::Matrix<double,15,12> G = Eigen::Matrix<double,15,12>::Zero();

    // Now compute Jacobian of new state wrt old state and noise
    if (state->options().do_fej) {

        // This is the change in the orientation from the end of the last prop to the current prop
        // This is needed since we need to include the "k-th" updated orientation information
        Eigen::Matrix<double,3,3> Rfej = state->imu()->Rot_fej();
        Eigen::Matrix<double,3,3> dR = quat_2_Rot(new_q)*Rfej.transpose();

        Eigen::Matrix<double,3,1> v_fej = state->imu()->vel_fej();
        Eigen::Matrix<double,3,1> p_fej = state->imu()->pos_fej();

        F.block(th_id, th_id, 3, 3) = dR;
        F.block(th_id, bg_id, 3, 3).noalias() = -dR * Jr_so3(-w_hat * dt) * dt;
        //F.block(th_id, bg_id, 3, 3).noalias() = -dR * Jr_so3(-log_so3(dR)) * dt;
        F.block(bg_id, bg_id, 3, 3).setIdentity();
        F.block(v_id, th_id, 3, 3).noalias() = -skew_x(new_v-v_fej+_gravity*dt)*Rfej.transpose();
        //F.block(v_id, th_id, 3, 3).noalias() = -Rfej.transpose() * skew_x(Rfej*(new_v-v_fej+_gravity*dt));
        F.block(v_id, v_id, 3, 3).setIdentity();
        F.block(v_id, ba_id, 3, 3) = -Rfej.transpose() * dt;
        F.block(ba_id, ba_id, 3, 3).setIdentity();
        F.block(p_id, th_id, 3, 3).noalias() = -skew_x(new_p-p_fej-v_fej*dt+0.5*_gravity*dt*dt)*Rfej.transpose();
        //F.block(p_id, th_id, 3, 3).noalias() = -0.5 * Rfej.transpose() * skew_x(2*Rfej*(new_p-p_fej-v_fej*dt+0.5*_gravity*dt*dt));
        F.block(p_id, v_id, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * dt;
        F.block(p_id, ba_id, 3, 3) = -0.5 * Rfej.transpose() * dt * dt;
        F.block(p_id, p_id, 3, 3).setIdentity();

        G.block(th_id, 0, 3, 3) = -dR * Jr_so3(-w_hat * dt) * dt;
        //G.block(th_id, 0, 3, 3) = -dR * Jr_so3(-log_so3(dR)) * dt;
        G.block(v_id, 3, 3, 3) = -Rfej.transpose() * dt;
        G.block(p_id, 3, 3, 3) = -0.5 * Rfej.transpose() * dt * dt;
        G.block(bg_id, 6, 3, 3) = dt*Eigen::Matrix<double,3,3>::Identity();
        G.block(ba_id, 9, 3, 3) = dt*Eigen::Matrix<double,3,3>::Identity();

    } else {

        Eigen::Matrix<double,3,3> R_Gtoi = state->imu()->Rot();

        F.block(th_id, th_id, 3, 3) = exp_so3(-w_hat * dt);
        F.block(th_id, bg_id, 3, 3).noalias() = -exp_so3(-w_hat * dt) * Jr_so3(-w_hat * dt) * dt;
        F.block(bg_id, bg_id, 3, 3).setIdentity();
        F.block(v_id, th_id, 3, 3).noalias() = -R_Gtoi.transpose() * skew_x(a_hat * dt);
        F.block(v_id, v_id, 3, 3).setIdentity();
        F.block(v_id, ba_id, 3, 3) = -R_Gtoi.transpose() * dt;
        F.block(ba_id, ba_id, 3, 3).setIdentity();
        F.block(p_id, th_id, 3, 3).noalias() = -0.5 * R_Gtoi.transpose() * skew_x(a_hat * dt * dt);
        F.block(p_id, v_id, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * dt;
        F.block(p_id, ba_id, 3, 3) = -0.5 * R_Gtoi.transpose() * dt * dt;
        F.block(p_id, p_id, 3, 3).setIdentity();

        G.block(th_id, 0, 3, 3) = -exp_so3(-w_hat * dt) * Jr_so3(-w_hat * dt) * dt;
        G.block(v_id, 3, 3, 3) = -R_Gtoi.transpose() * dt;
        G.block(p_id, 3, 3, 3) = -0.5 * R_Gtoi.transpose() * dt * dt;
        G.block(bg_id, 6, 3, 3) = dt*Eigen::Matrix<double,3,3>::Identity();
        G.block(ba_id, 9, 3, 3) = dt*Eigen::Matrix<double,3,3>::Identity();
    }

    // Construct our discrete noise covariance matrix
    // Note that we need to convert our continuous time noises to discrete
    // Equations (129) amd (130) of Trawny tech report
    Eigen::Matrix<double,12,12> Qc = Eigen::Matrix<double,12,12>::Zero();
    Qc.block(0,0,3,3) = _noises.sigma_w_2/dt*Eigen::Matrix<double,3,3>::Identity();
    Qc.block(3,3,3,3) = _noises.sigma_a_2/dt*Eigen::Matrix<double,3,3>::Identity();
    Qc.block(6,6,3,3) = _noises.sigma_wb_2/dt*Eigen::Matrix<double,3,3>::Identity();
    Qc.block(9,9,3,3) = _noises.sigma_ab_2/dt*Eigen::Matrix<double,3,3>::Identity();

    // Compute the noise injected into the state over the interval
    Qd = G*Qc*G.transpose();
    Qd = 0.5*(Qd+Qd.transpose());

    //Now replace imu estimate and fej with propagated values
    Eigen::Matrix<double,16,1> imu_x = state->imu()->value();
    imu_x.block(0,0,4,1) = new_q;
    imu_x.block(4,0,3,1) = new_p;
    imu_x.block(7,0,3,1) = new_v;
    state->imu()->set_value(imu_x);                                         // 已经修改了当前的IMU状态
    state->imu()->set_fej(imu_x);                                           // 状态向量的值已经向前传播了一段时间

}


void Propagator::predict_mean_discrete(State *state, double dt,
                                        const Eigen::Vector3d &w_hat1, const Eigen::Vector3d &a_hat1,
                                        const Eigen::Vector3d &w_hat2, const Eigen::Vector3d &a_hat2,
                                        Eigen::Vector4d &new_q, Eigen::Vector3d &new_v, Eigen::Vector3d &new_p) {

    // If we are averaging the IMU, then do so
    Eigen::Vector3d w_hat = w_hat1;
    Eigen::Vector3d a_hat = a_hat1;
    if (state->options().imu_avg) {
        w_hat = .5*(w_hat1+w_hat2);
        a_hat = .5*(a_hat1+a_hat2);
    }

    // Pre-compute things
    double w_norm = w_hat.norm();
    Eigen::Matrix<double,4,4> I_4x4 = Eigen::Matrix<double,4,4>::Identity();
    Eigen::Matrix<double,3,3> R_Gtoi = state->imu()->Rot();

    // Orientation: Equation (101) and (103) and of Trawny indirect TR
    Eigen::Matrix<double,4,4> bigO;
    if(w_norm > 1e-20) {
        bigO = cos(0.5*w_norm*dt)*I_4x4 + 1/w_norm*sin(0.5*w_norm*dt)*Omega(w_hat);
    } else {
        bigO = I_4x4 + 0.5*dt*Omega(w_hat);
    }
    new_q = quatnorm(bigO*state->imu()->quat());
    //new_q = rot_2_quat(exp_so3(-w_hat*dt)*R_Gtoi);

    // Velocity: just the acceleration in the local frame, minus global gravity
    new_v = state->imu()->vel() + R_Gtoi.transpose()*a_hat*dt - _gravity*dt;

    // Position: just velocity times dt, with the acceleration integrated twice
    new_p = state->imu()->pos() + state->imu()->vel()*dt + 0.5*R_Gtoi.transpose()*a_hat*dt*dt - 0.5*_gravity*dt*dt;

}



void Propagator::predict_mean_rk4(State *state, double dt,
                                  const Eigen::Vector3d &w_hat1, const Eigen::Vector3d &a_hat1,
                                  const Eigen::Vector3d &w_hat2, const Eigen::Vector3d &a_hat2,
                                  Eigen::Vector4d &new_q, Eigen::Vector3d &new_v, Eigen::Vector3d &new_p) {

    // Pre-compute things
    Eigen::Vector3d w_hat = w_hat1;
    Eigen::Vector3d a_hat = a_hat1;
    Eigen::Vector3d w_alpha = (w_hat2-w_hat1)/dt;
    Eigen::Vector3d a_jerk = (a_hat2-a_hat1)/dt;

    // y0 ================
    Eigen::Vector4d q_0 = state->imu()->quat();
    Eigen::Vector3d p_0 = state->imu()->pos();
    Eigen::Vector3d v_0 = state->imu()->vel();

    // k1 ================
    Eigen::Vector4d dq_0 = {0,0,0,1};
    Eigen::Vector4d q0_dot = 0.5*Omega(w_hat)*dq_0;
    Eigen::Vector3d p0_dot = v_0;
    Eigen::Matrix3d R_Gto0 = quat_2_Rot(quat_multiply(dq_0,q_0));
    Eigen::Vector3d v0_dot = R_Gto0.transpose()*a_hat-_gravity;

    Eigen::Vector4d k1_q = q0_dot*dt;
    Eigen::Vector3d k1_p = p0_dot*dt;
    Eigen::Vector3d k1_v = v0_dot*dt;

    // k2 ================
    w_hat += 0.5*w_alpha*dt;
    a_hat += 0.5*a_jerk*dt;

    Eigen::Vector4d dq_1 = quatnorm(dq_0+0.5*k1_q);
    //Eigen::Vector3d p_1 = p_0+0.5*k1_p;
    Eigen::Vector3d v_1 = v_0+0.5*k1_v;

    Eigen::Vector4d q1_dot = 0.5*Omega(w_hat)*dq_1;
    Eigen::Vector3d p1_dot = v_1;
    Eigen::Matrix3d R_Gto1 = quat_2_Rot(quat_multiply(dq_1,q_0));
    Eigen::Vector3d v1_dot = R_Gto1.transpose()*a_hat-_gravity;

    Eigen::Vector4d k2_q = q1_dot*dt;
    Eigen::Vector3d k2_p = p1_dot*dt;
    Eigen::Vector3d k2_v = v1_dot*dt;

    // k3 ================
    Eigen::Vector4d dq_2 = quatnorm(dq_0+0.5*k2_q);
    //Eigen::Vector3d p_2 = p_0+0.5*k2_p;
    Eigen::Vector3d v_2 = v_0+0.5*k2_v;

    Eigen::Vector4d q2_dot = 0.5*Omega(w_hat)*dq_2;
    Eigen::Vector3d p2_dot = v_2;
    Eigen::Matrix3d R_Gto2 = quat_2_Rot(quat_multiply(dq_2,q_0));
    Eigen::Vector3d v2_dot = R_Gto2.transpose()*a_hat-_gravity;

    Eigen::Vector4d k3_q = q2_dot*dt;
    Eigen::Vector3d k3_p = p2_dot*dt;
    Eigen::Vector3d k3_v = v2_dot*dt;

    // k4 ================
    w_hat += 0.5*w_alpha*dt;
    a_hat += 0.5*a_jerk*dt;

    Eigen::Vector4d dq_3 = quatnorm(dq_0+k3_q);
    //Eigen::Vector3d p_3 = p_0+k3_p;
    Eigen::Vector3d v_3 = v_0+k3_v;

    Eigen::Vector4d q3_dot = 0.5*Omega(w_hat)*dq_3;
    Eigen::Vector3d p3_dot = v_3;
    Eigen::Matrix3d R_Gto3 = quat_2_Rot(quat_multiply(dq_3,q_0));
    Eigen::Vector3d v3_dot = R_Gto3.transpose()*a_hat-_gravity;

    Eigen::Vector4d k4_q = q3_dot*dt;
    Eigen::Vector3d k4_p = p3_dot*dt;
    Eigen::Vector3d k4_v = v3_dot*dt;

    // y+dt ================
    Eigen::Vector4d dq = quatnorm(dq_0+(1.0/6.0)*k1_q+(1.0/3.0)*k2_q+(1.0/3.0)*k3_q+(1.0/6.0)*k4_q);
    new_q = quat_multiply(dq, q_0);
    new_p = p_0+(1.0/6.0)*k1_p+(1.0/3.0)*k2_p+(1.0/3.0)*k3_p+(1.0/6.0)*k4_p;
    new_v = v_0+(1.0/6.0)*k1_v+(1.0/3.0)*k2_v+(1.0/3.0)*k3_v+(1.0/6.0)*k4_v;

}



