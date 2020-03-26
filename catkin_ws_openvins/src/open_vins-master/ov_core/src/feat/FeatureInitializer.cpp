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
#include "FeatureInitializer.h"
#include <iostream>
#include <iomanip>
using namespace ov_core;



// 直接参照图片三角化逆深度说明
// clonesCAM 一一个参数为相机id,第二个参数为相机pose
// OpenVINS并没有说直接用双目的匹配初始化特征点3D坐标值，而是把双目看成两个相对关系的单目，然后用更多两两配对的单目（已知位姿）构成的sfm问题来求解特征点的3D坐标。
double FeatureInitializer::compute_error(std::unordered_map<size_t,std::unordered_map<double,ClonePose>> &clonesCAM,
                                         Feature* feat, double alpha, double beta, double rho) {

    // Total error
    double err = 0;

    // Get the position of the anchor pose   // 得到局部参考帧的位姿
    Eigen::Matrix<double,3,3> &R_GtoA = clonesCAM.at(feat->anchor_cam_id).at(feat->anchor_clone_timestamp).Rot();
    Eigen::Matrix<double,3,1> &p_AinG = clonesCAM.at(feat->anchor_cam_id).at(feat->anchor_clone_timestamp).pos();

    // Loop through each camera for this feature
    for (auto const& pair : feat->timestamps) {
        // Add CAM_I features
        for (size_t m = 0; m < feat->timestamps.at(pair.first).size(); m++) {

            //=====================================================================================
            //=====================================================================================

            // Get the position of this clone in the global   相对位姿变换
            Eigen::Matrix<double, 3, 3> &R_GtoCi = clonesCAM.at(pair.first).at(feat->timestamps.at(pair.first).at(m)).Rot();
            Eigen::Matrix<double, 3, 1> &p_CiinG = clonesCAM.at(pair.first).at(feat->timestamps.at(pair.first).at(m)).pos();
            // Convert current position relative to anchor        得到所有观测帧到局部参考帧的位姿变换
            Eigen::Matrix<double,3,3> R_AtoCi;
            R_AtoCi.noalias() = R_GtoCi*R_GtoA.transpose();
            Eigen::Matrix<double,3,1> p_CiinA;
            p_CiinA.noalias() = R_GtoA*(p_CiinG-p_AinG);
            Eigen::Matrix<double,3,1> p_AinCi;
            p_AinCi.noalias() = -R_AtoCi*p_CiinA;

            //=====================================================================================
            //=====================================================================================

            // Middle variables of the system  中间变量
            double hi1 = R_AtoCi(0, 0) * alpha + R_AtoCi(0, 1) * beta + R_AtoCi(0, 2) + rho * p_AinCi(0, 0);
            double hi2 = R_AtoCi(1, 0) * alpha + R_AtoCi(1, 1) * beta + R_AtoCi(1, 2) + rho * p_AinCi(1, 0);
            double hi3 = R_AtoCi(2, 0) * alpha + R_AtoCi(2, 1) * beta + R_AtoCi(2, 2) + rho * p_AinCi(2, 0);
            // Calculate residual
            Eigen::Matrix<float, 2, 1> z;
            z << hi1 / hi3, hi2 / hi3;
            Eigen::Matrix<float, 2, 1> res = feat->uvs_norm.at(pair.first).at(m) - z;            // uvs_norm 表示归一化平面坐标
            // Append to our summation variables
            err += pow(res.norm(), 2);
        }
    }

    return err;

}


// 三角化得到 feat->p_FinA = p_f;feat->p_FinG
// std::unordered_map<size_t,std::unordered_map<double,ClonePose>> &clonesCAM
//                    camera_id                 timestamp,pose
bool FeatureInitializer::single_triangulation(Feature* feat, std::unordered_map<size_t,std::unordered_map<double,ClonePose>> &clonesCAM) {


    // Total number of measurements
    // Also set the first measurement to be the anchor frame
    int total_meas = 0;
    size_t anchor_most_meas = 0;                 // camera_id
    size_t most_meas = 0;
    for (auto const& pair : feat->timestamps) {  // pair camera_id std::vector<double> t
        total_meas += (int)pair.second.size();   // 所有相机总共的观测数量
        if(pair.second.size() > most_meas) {
            anchor_most_meas = pair.first;       // 最大观测对应的相机id
            most_meas = pair.second.size();      // 最大的观测数目
        }
    }
    feat->anchor_cam_id = anchor_most_meas;
    feat->anchor_clone_timestamp = feat->timestamps.at(feat->anchor_cam_id).back();
//    feat->anchor_clone_timestamp = feat->timestamps.at(feat->anchor_cam_id)[0];
    // 拥有最大观测数目的相机所对应观测的最后一帧img时间戳 ??? 最后一个观测的img作为局部坐标系 ??? first observation
//    std::cout <<"back::::::::::::::::::::"<< std::endl;
//    std::cout << std::fixed << setprecision(9) <<feat->timestamps.at(feat->anchor_cam_id).back()<<std::endl;
//    std::cout <<"first::::::::::::::::::::"<< std::endl;
//    std::cout << std::fixed << setprecision(9) <<feat->timestamps.at(feat->anchor_cam_id)[0]<<std::endl;
//    std::cout << std::setprecision(19)<<"first::::::::::::::::::::"<<feat->timestamps.at(feat->anchor_cam_id)[0]<<std::endl;
    // Our linear system matrices
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2*total_meas, 3);
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(2*total_meas, 1);

    // Location in the linear system matrices
    size_t c = 0;

    // Get the position of the anchor pose
    ClonePose anchorclone = clonesCAM.at(feat->anchor_cam_id).at(feat->anchor_clone_timestamp);
    Eigen::Matrix<double,3,3> &R_GtoA = anchorclone.Rot();
    Eigen::Matrix<double,3,1> &p_AinG = anchorclone.pos();

    // Loop through each camera for this feature
    for (auto const& pair : feat->timestamps) {

        // Add CAM_I features
        for (size_t m = 0; m < feat->timestamps.at(pair.first).size(); m++) {

            // Get the position of this clone in the global
            Eigen::Matrix<double, 3, 3> &R_GtoCi = clonesCAM.at(pair.first).at(feat->timestamps.at(pair.first).at(m)).Rot();
            Eigen::Matrix<double, 3, 1> &p_CiinG = clonesCAM.at(pair.first).at(feat->timestamps.at(pair.first).at(m)).pos();

            // Convert current position relative to anchor
            Eigen::Matrix<double,3,3> R_AtoCi;
            R_AtoCi.noalias() = R_GtoCi*R_GtoA.transpose();
            Eigen::Matrix<double,3,1> p_CiinA;
            p_CiinA.noalias() = R_GtoA*(p_CiinG-p_AinG);

            // Get the UV coordinate normal
            Eigen::Matrix<double, 3, 1> b_i;
            b_i << feat->uvs_norm.at(pair.first).at(m)(0), feat->uvs_norm.at(pair.first).at(m)(1), 1;
            b_i = R_AtoCi.transpose() * b_i;
            b_i = b_i / b_i.norm();
            Eigen::Matrix<double,2,3> Bperp = Eigen::Matrix<double,2,3>::Zero();
            Bperp << -b_i(2, 0), 0, b_i(0, 0), 0, b_i(2, 0), -b_i(1, 0);                          // 投影到左零空间求解消除深度信息的影响

            // Append to our linear system
            A.block(2 * c, 0, 2, 3) = Bperp;
            b.block(2 * c, 0, 2, 1).noalias() = Bperp * p_CiinA;
            c++;

        }
    }

    // Solve the linear system
    Eigen::MatrixXd p_f = A.colPivHouseholderQr().solve(b);   // QR分解 Ax=b 的未知数

    // Check A and p_f
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd singularValues;
    singularValues.resize(svd.singularValues().rows(), 1);
    singularValues = svd.singularValues();
    double condA = singularValues(0, 0) / singularValues(singularValues.rows() - 1, 0);

    // If we have a bad condition number, or it is too close
    // Then set the flag for bad (i.e. set z-axis to nan)

    // z最近距离0.25m  z最远距离40m 条件数 1000
    if (std::abs(condA) > _options.max_cond_number || p_f(2,0) < _options.min_dist || p_f(2,0) > _options.max_dist || std::isnan(p_f.norm())){
        return false;
    }

    // Store it in our feature object
    feat->p_FinA = p_f;
    feat->p_FinG = R_GtoA.transpose()*feat->p_FinA + p_AinG;
    return true;

}



bool FeatureInitializer::single_gaussnewton(Feature* feat, std::unordered_map<size_t,std::unordered_map<double,ClonePose>> &clonesCAM) {

    //Get into inverse depth
    double rho = 1/feat->p_FinA(2);
    double alpha = feat->p_FinA(0)/feat->p_FinA(2);
    double beta = feat->p_FinA(1)/feat->p_FinA(2);

    // Optimization parameters
    double lam = _options.init_lamda;
    double eps = 10000;
    int runs = 0;

    // Variables used in the optimization
    bool recompute = true;
    Eigen::Matrix<double,3,3> Hess = Eigen::Matrix<double,3,3>::Zero();
    Eigen::Matrix<double,3,1> grad = Eigen::Matrix<double,3,1>::Zero();

    // Cost at the last iteration
    double cost_old = compute_error(clonesCAM,feat,alpha,beta,rho);        // 计算当前的残差

    // Get the position of the anchor pose
    Eigen::Matrix<double,3,3> &R_GtoA = clonesCAM.at(feat->anchor_cam_id).at(feat->anchor_clone_timestamp).Rot();
    Eigen::Matrix<double,3,1> &p_AinG = clonesCAM.at(feat->anchor_cam_id).at(feat->anchor_clone_timestamp).pos();

    // Loop till we have either
    // 1. Reached our max iteration count
    // 2. System is unstable
    // 3. System has converged    收敛
    while (runs < _options.max_runs && lam < _options.max_lamda && eps > _options.min_dx) {

        // Triggers a recomputation of jacobians/information/gradients
        if (recompute) {

            Hess.setZero();
            grad.setZero();

            double err = 0;

            // Loop through each camera for this feature
            for (auto const& pair : feat->timestamps) {

                // Add CAM_I features
                for (size_t m = 0; m < feat->timestamps.at(pair.first).size(); m++) {

                    //=====================================================================================
                    //=====================================================================================

                    // Get the position of this clone in the global
                    Eigen::Matrix<double, 3, 3> &R_GtoCi = clonesCAM.at(pair.first).at(feat->timestamps[pair.first].at(m)).Rot();
                    Eigen::Matrix<double, 3, 1> &p_CiinG = clonesCAM.at(pair.first).at(feat->timestamps[pair.first].at(m)).pos();
                    // Convert current position relative to anchor
                    Eigen::Matrix<double,3,3> R_AtoCi;
                    R_AtoCi.noalias() = R_GtoCi*R_GtoA.transpose();
                    Eigen::Matrix<double,3,1> p_CiinA;
                    p_CiinA.noalias() = R_GtoA*(p_CiinG-p_AinG);
                    Eigen::Matrix<double,3,1> p_AinCi;
                    p_AinCi.noalias() = -R_AtoCi*p_CiinA;

                    //=====================================================================================
                    //=====================================================================================

                    // Middle variables of the system
                    double hi1 = R_AtoCi(0, 0) * alpha + R_AtoCi(0, 1) * beta + R_AtoCi(0, 2) + rho * p_AinCi(0, 0);
                    double hi2 = R_AtoCi(1, 0) * alpha + R_AtoCi(1, 1) * beta + R_AtoCi(1, 2) + rho * p_AinCi(1, 0);
                    double hi3 = R_AtoCi(2, 0) * alpha + R_AtoCi(2, 1) * beta + R_AtoCi(2, 2) + rho * p_AinCi(2, 0);
                    // Calculate jacobian
                    double d_z1_d_alpha = (R_AtoCi(0, 0) * hi3 - hi1 * R_AtoCi(2, 0)) / (pow(hi3, 2));
                    double d_z1_d_beta = (R_AtoCi(0, 1) * hi3 - hi1 * R_AtoCi(2, 1)) / (pow(hi3, 2));
                    double d_z1_d_rho = (p_AinCi(0, 0) * hi3 - hi1 * p_AinCi(2, 0)) / (pow(hi3, 2));
                    double d_z2_d_alpha = (R_AtoCi(1, 0) * hi3 - hi2 * R_AtoCi(2, 0)) / (pow(hi3, 2));
                    double d_z2_d_beta = (R_AtoCi(1, 1) * hi3 - hi2 * R_AtoCi(2, 1)) / (pow(hi3, 2));
                    double d_z2_d_rho = (p_AinCi(1, 0) * hi3 - hi2 * p_AinCi(2, 0)) / (pow(hi3, 2));
                    Eigen::Matrix<double, 2, 3> H;
                    H << d_z1_d_alpha, d_z1_d_beta, d_z1_d_rho, d_z2_d_alpha, d_z2_d_beta, d_z2_d_rho;
                    // Calculate residual
                    Eigen::Matrix<float, 2, 1> z;
                    z << hi1 / hi3, hi2 / hi3;
                    Eigen::Matrix<float, 2, 1> res = feat->uvs_norm.at(pair.first).at(m) - z;

                    //=====================================================================================
                    //=====================================================================================

                    // Append to our summation variables
                    err += std::pow(res.norm(), 2);
                    grad.noalias() += H.transpose() * res.cast<double>();
                    Hess.noalias() += H.transpose() * H;
                }

            }

        }

        // Solve Levenberg iteration
        Eigen::Matrix<double,3,3> Hess_l = Hess;
        for (size_t r=0; r < (size_t)Hess.rows(); r++){
            Hess_l(r,r) *= (1.0+lam);
        }

        Eigen::Matrix<double,3,1> dx = Hess_l.colPivHouseholderQr().solve(grad);
        //Eigen::Matrix<double,3,1> dx = (Hess+lam*Eigen::MatrixXd::Identity(Hess.rows(), Hess.rows())).colPivHouseholderQr().solve(grad);

        // Check if error has gone down
        double cost = compute_error(clonesCAM,feat,alpha+dx(0,0),beta+dx(1,0),rho+dx(2,0));  // 在逆深度都加上对应的增量后检查cost有没有下降

        // Debug print
        //cout << "run = " << runs << " | cost = " << dx.norm() << " | lamda = " << lam << " | depth = " << 1/rho << endl;

        // Check if converged
        if (cost <= cost_old && (cost_old-cost)/cost_old < _options.min_dcost) {    // 达到收敛条件，退出求解
            alpha += dx(0, 0);
            beta += dx(1, 0);
            rho += dx(2, 0);
            eps = 0;
            break;
        }

        // If cost is lowered, accept step
        // Else inflate lambda (try to make more stable)
        if (cost <= cost_old) {                   // cost减小了，但是没有达到达到收敛条件，更新步长，减小lamda，继续求解
            recompute = true;
            cost_old = cost;
            alpha += dx(0, 0);
            beta += dx(1, 0);
            rho += dx(2, 0);
            runs++;
            lam = lam/_options.lam_mult;
            eps = dx.norm();
        } else {
            recompute = false;                 // cost 增大了，不更新步长，不用重新求解Hess和Error，增大lamda，继续求解
            lam = lam*_options.lam_mult;
            continue;
        }
    }

    // Revert to standard, and set to all
    feat->p_FinA(0) = alpha/rho;                   // 达到收敛条件，退出循环将参数转换为参考局部帧的坐标
    feat->p_FinA(1) = beta/rho;
    feat->p_FinA(2) = 1/rho;

    // Get tangent plane to x_hat
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(feat->p_FinA);
    Eigen::MatrixXd Q = qr.householderQ();

    // Max baseline we have between poses
    double base_line_max = 0.0;

    // Check maximum baseline
    // Loop through each camera for this feature
    for (auto const& pair : feat->timestamps) {
        // Loop through the other clones to see what the max baseline is
        for (size_t m = 0; m < feat->timestamps.at(pair.first).size(); m++) {
            // Get the position of this clone in the global
            Eigen::Matrix<double,3,1> &p_CiinG  = clonesCAM.at(pair.first).at(feat->timestamps.at(pair.first).at(m)).pos();
            // Convert current position relative to anchor
            Eigen::Matrix<double,3,1> p_CiinA = R_GtoA*(p_CiinG-p_AinG);
            // Dot product camera pose and nullspace     // 左零空间
            double base_line = ((Q.block(0,1,3,2)).transpose() * p_CiinA).norm();
            if (base_line > base_line_max) base_line_max = base_line;
        }
    }

    // Check if this feature is bad or not
    // 1. If the feature is too close
    // 2. If the feature is invalid
    // 3. If the baseline ratio is large
    if(feat->p_FinA(2) < _options.min_dist
        || feat->p_FinA(2) > _options.max_dist
        || (feat->p_FinA.norm() / base_line_max) > _options.max_baseline
        || std::isnan(feat->p_FinA.norm())) {
        return false;
    }

    // Finally get position in global frame
    feat->p_FinG = R_GtoA.transpose()*feat->p_FinA + p_AinG;
    return true;

}