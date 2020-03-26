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
#include "UpdaterMSCKF.h"



using namespace ov_core;
using namespace ov_msckf;




// feature_vec 中存储的是待更新的MSCKF特征点，经经剥离开的
void UpdaterMSCKF::update(State *state, std::vector<Feature*>& feature_vec) {

    // Return if no features   没有待更新的特征点直接返回
    if(feature_vec.empty())
        return;

    // Start timing
    boost::posix_time::ptime rT0, rT1, rT2, rT3, rT4, rT5, rT6, rT7;
    rT0 =  boost::posix_time::microsec_clock::local_time();

    // 0. Get all timestamps our clones are at (and thus valid measurement times)
    std::vector<double> clonetimes;                             // 获取IMU状态的时间戳
    for(const auto& clone_imu : state->get_clones()) {          // IMU时间戳对齐
        clonetimes.emplace_back(clone_imu.first);
    }

    // 去除不在IMU状态的跟踪特征点，只关注IMU状态时间戳对应的那些特征点就足够了
    // 1. Clean all feature measurements and make sure they all have valid clone times
    auto it0 = feature_vec.begin();
    while(it0 != feature_vec.end()) {

        // Clean the feature
        (*it0)->clean_old_measurements(clonetimes);

        // Count how many measurements
        int ct_meas = 0;
        for(const auto &pair : (*it0)->timestamps) {       // pair 第一个参数是观测相机id,第二个参数是时间戳，对左右相机的观测汇总到一起
            ct_meas += (*it0)->timestamps[pair.first].size();
        }

        // Remove if we don't have enough
        if(ct_meas < 3) {                                 // 如果某一个特征点的观测数目小于3，直接删除
            (*it0)->to_delete = true;
            it0 = feature_vec.erase(it0);
        } else {
            it0++;
        }

    }
    rT1 =  boost::posix_time::microsec_clock::local_time();

    // 2. Create vector of cloned *CAMERA* poses at each of our clone timesteps
    // 对左右相机，根据滑窗内的所有IMU位姿，推算每对相机的位姿，将左右相机的clone的位姿存储起来用于三角化
    // size_t 表示机机id，第二个double 表示时间戳，clone_pose 表示对应的clonepose
    std::unordered_map<size_t, std::unordered_map<double, FeatureInitializer::ClonePose>> clones_cam;
    for(const auto &clone_calib : state->get_calib_IMUtoCAMs()) {

        // For this camera, create the vector of camera poses
        std::unordered_map<double, FeatureInitializer::ClonePose> clones_cami;
        for(const auto &clone_imu : state->get_clones()) {

            // Get current camera pose                    R_IitoCi     *       R_GtoIi              // Ii 表示第i个clone对应的IMU
            Eigen::Matrix<double,3,3> R_GtoCi = clone_calib.second->Rot()*clone_imu.second->Rot();
            Eigen::Matrix<double,3,1> p_CioinG = clone_imu.second->pos() - R_GtoCi.transpose()*clone_calib.second->pos();

            // Append to our map          first 表示时戳戳，
            clones_cami.insert({clone_imu.first,FeatureInitializer::ClonePose(R_GtoCi,p_CioinG)});

        }

        // Append to our map
        clones_cam.insert({clone_calib.first,clones_cami});

    }

    //（已知滑窗内的所有相机位姿和特征点的2D匹配点坐标），先三角化得到初始值，然后再进行高斯非线性进一步优化（类似BA）
    // clones_cam的第一个元素表示左右相机的标识，第二个元素表示位姿，对所有的特征点点都进行了三角化
    // 3. Try to triangulate all MSCKF or new SLAM features that have measurements
    auto it1 = feature_vec.begin();
    while(it1 != feature_vec.end()) {

        // Triangulate the feature and remove if it fails
        bool success = initializer_feat->single_triangulation(*it1, clones_cam);  // 线性三角化得到3D点的初始值
        if(!success) {
            (*it1)->to_delete = true;
            it1 = feature_vec.erase(it1);                                         // 三角化不成功直接剔除这个点
            continue;
        }

        // Gauss-newton refine the feature
        success = initializer_feat->single_gaussnewton(*it1, clones_cam);         // 高斯牛顿得到优化值
        if(!success) {
            (*it1)->to_delete = true;
            it1 = feature_vec.erase(it1);
            continue;
        }
        it1++;

    }
    rT2 =  boost::posix_time::microsec_clock::local_time();


    // Calculate the max possible measurement size，计算最大可能性的观测值
    size_t max_meas_size = 0;
    for(size_t i=0; i<feature_vec.size(); i++) {                            // 现在的feature_vector 表示成功三角化的MSCKF特征点
        for (const auto &pair : feature_vec.at(i)->timestamps) {            // pair 表示左右相机
            max_meas_size += 2*feature_vec.at(i)->timestamps[pair.first].size();
        }
    }

    // Calculate max possible state size (i.e. the size of our covariance)   计算最大可能性的hx
    size_t max_hx_size = state->n_vars();                                 // msckf 协方差长度，当前所有状态量的维度
    max_hx_size -= 3*state->features_SLAM().size();                       // slam 特征点个数乘3，减去slam_feature的维度，slam_feature自由度为3

    // Large Jacobian and residual of *all* features for this update
    Eigen::VectorXd res_big = Eigen::VectorXd::Zero(max_meas_size);             // 2M_j 所有特征点整个的残差
    Eigen::MatrixXd Hx_big = Eigen::MatrixXd::Zero(max_meas_size, max_hx_size);     // 2M_j * L
    std::unordered_map<Type*,size_t> Hx_mapping;
    std::vector<Type*> Hx_order_big;
    size_t ct_jacob = 0;
    size_t ct_meas = 0;


    // 4. Compute linear system for each feature, nullspace project, and reject
    auto it2 = feature_vec.begin();
    while(it2 != feature_vec.end()) {

        // Convert our feature into our current format        设置特征点的属性进行更新
        UpdaterHelper::UpdaterHelperFeature feat;
        feat.featid = (*it2)->featid;
        feat.uvs = (*it2)->uvs;
        feat.uvs_norm = (*it2)->uvs_norm;
        feat.timestamps = (*it2)->timestamps;
        feat.feat_representation = state->options().feat_representation;

        // Save the position and its fej value     根据特征点的类型，选择拷贝的是anchor 还是全局信息：
        if(FeatureRepresentation::is_relative_representation(feat.feat_representation)) {
            feat.anchor_cam_id = (*it2)->anchor_cam_id;
            feat.anchor_clone_timestamp = (*it2)->anchor_clone_timestamp;
            feat.p_FinA = (*it2)->p_FinA;
            feat.p_FinA_fej = (*it2)->p_FinA;
        } else {
            feat.p_FinG = (*it2)->p_FinG;
            feat.p_FinG_fej = (*it2)->p_FinG;
        }

        // H_x和 Hx_order是配对的，前者存储了观测特征点关于MSCKF状态的导数，后者存储了MSCKF状态和大小
        // Our return values (feature jacobian, state jacobian, residual, and order of state jacobian)
        Eigen::MatrixXd H_f;     // 对当前特征点的导数
        Eigen::MatrixXd H_x;     // 当前特征点的观测对状态向量的导数
        Eigen::VectorXd res;     // 当前特征点的残差
        std::vector<Type*> Hx_order;   // 当前msckf的状态和大小

        // Get the Jacobian for this feature  返回Hx 和Hf，即为观测关于IMU姿态，IMU到相机的外参数，相机内参数和特征点图像坐标的Jacobian矩阵
        UpdaterHelper::get_feature_jacobian_full(state, feat, H_f, H_x, res, Hx_order);

        // Nullspace project       左零空间影影
        UpdaterHelper::nullspace_project_inplace(H_f, H_x, res);

        /// Chi2 distance check     卡方检验
        Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
        Eigen::MatrixXd S = H_x*P_marg*H_x.transpose();
        S.diagonal() += _options.sigma_pix_sq*Eigen::VectorXd::Ones(S.rows());
        double chi2 = res.dot(S.llt().solve(res));

        // Get our threshold (we precompute up to 500 but handle the case that it is more)
        double chi2_check;
        if(res.rows() < 500) {             // 若长度小于500 （预设的自由度）则直接调用事先初始化过的卡方查询表
            chi2_check = chi_squared_table[res.rows()];
        } else {
            boost::math::chi_squared chi_squared_dist(res.rows());            // 否则重新创建一个长度一致的卡方查询表
            chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
            std::cout << "chi2_check over the residual limit - " << res.rows() << std::endl;
        }

        // Check if we should delete or not  不满足卡方校验直接删除
        if(chi2 > _options.chi2_multipler*chi2_check) {     // 若超过卡方检验的值，说明这个特征点不可靠，可以删除
            (*it2)->to_delete = true;
            it2 = feature_vec.erase(it2);
            //cout << "featid = " << feat.featid << endl;
            //cout << "chi2 = " << chi2 << " > " << _options.chi2_multipler*chi2_check << endl;
            //cout << "res = " << endl << res.transpose() << endl;
            continue;
        }

        // We are good!!! Append to our large H vector
        size_t ct_hx = 0;
        for(const auto &var : Hx_order) {

            // Ensure that this variable is in our Jacobian
            if(Hx_mapping.find(var)==Hx_mapping.end()) {
                Hx_mapping.insert({var,ct_jacob});
                Hx_order_big.push_back(var);
                ct_jacob += var->size();
            }

            // Append to our large Jacobian
            Hx_big.block(ct_meas,Hx_mapping[var],H_x.rows(),var->size()) = H_x.block(0,ct_hx,H_x.rows(),var->size());
            ct_hx += var->size();

        }

        // Append our residual and move forward
        res_big.block(ct_meas,0,res.rows(),1) = res;
        ct_meas += res.rows();
        it2++;

    }
    rT3 =  boost::posix_time::microsec_clock::local_time();

    // We have appended all features to our Hx_big, res_big
    // Delete it so we do not reuse information
    // 将所有特征点的信息整合进hx中，然后设置特征点的状态to_delete为true,表示待删除状态
    for (size_t f=0; f < feature_vec.size(); f++){
        feature_vec[f]->to_delete = true;
    }

    // Return if we don't have anything and resize our matrices
    if(ct_meas < 1) {
        return;
    }
    assert(ct_meas<=max_meas_size);
    assert(ct_jacob<=max_hx_size);
    res_big.conservativeResize(ct_meas,1);
    Hx_big.conservativeResize(ct_meas,ct_jacob);

    // 5. Perform measurement compression         QR分解压缩矩阵
    UpdaterHelper::measurement_compress_inplace(Hx_big, res_big);
    if(Hx_big.rows() < 1) {
        return;
    }
    rT4 =  boost::posix_time::microsec_clock::local_time();


    // Our noise is isotropic, so make it here after our compression 设置更新的高斯白噪声协方差
    Eigen::MatrixXd R_big = _options.sigma_pix_sq*Eigen::MatrixXd::Identity(res_big.rows(),res_big.rows());

    // 6. With all good features update the state    对所有好的特征点进行EKF更新
    StateHelper::EKFUpdate(state, Hx_order_big, Hx_big, res_big, R_big);
    rT5 =  boost::posix_time::microsec_clock::local_time();

    // Debug print timing information
    ROS_INFO("[MSCKF-UP]: %.4f seconds to clean",(rT1-rT0).total_microseconds() * 1e-6);
    ROS_INFO("[MSCKF-UP]: %.4f seconds to triangulate",(rT2-rT1).total_microseconds() * 1e-6);
    ROS_INFO("[MSCKF-UP]: %.4f seconds create system (%d features)",(rT3-rT2).total_microseconds() * 1e-6, (int)feature_vec.size());
    ROS_INFO("[MSCKF-UP]: %.4f seconds compress system",(rT4-rT3).total_microseconds() * 1e-6);
    ROS_INFO("[MSCKF-UP]: %.4f seconds update state (%d size)",(rT5-rT4).total_microseconds() * 1e-6, (int)res_big.rows());
    ROS_INFO("[MSCKF-UP]: %.4f seconds total",(rT5-rT1).total_microseconds() * 1e-6);

}










