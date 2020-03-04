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
#include "StateHelper.h"


using namespace ov_core;
using namespace ov_msckf;




void StateHelper::EKFUpdate(State *state, const std::vector<Type *> &H_order, const Eigen::MatrixXd &H,
                            const Eigen::VectorXd &res, const Eigen::MatrixXd &R) {

    //==========================================================
    //==========================================================
    // Part of the Kalman Gain K = (P*H^T)*S^{-1} = M*S^{-1}
    assert(res.rows() == R.rows());
    assert(H.rows() == res.rows());
    Eigen::MatrixXd M_a = Eigen::MatrixXd::Zero(state->n_vars(), res.rows());

    // Get the location in small jacobian for each measuring variable
    int current_it = 0;
    std::vector<int> H_id;
    for (Type *meas_var: H_order) {
        H_id.push_back(current_it);
        current_it += meas_var->size();
    }

    auto &Cov = state->Cov();

    //==========================================================
    //==========================================================
    // For each active variable find its M = P*H^T
    for (Type *var: state->variables()) {
        // Sum up effect of each subjacobian = K_i= \sum_m (P_im Hm^T)
        Eigen::MatrixXd M_i = Eigen::MatrixXd::Zero(var->size(), res.rows());
        for (size_t i = 0; i < H_order.size(); i++) {
            Type *meas_var = H_order[i];
            M_i.noalias() += Cov.block(var->id(), meas_var->id(), var->size(), meas_var->size()) *
                             H.block(0, H_id[i], H.rows(), meas_var->size()).transpose();
        }
        M_a.block(var->id(), 0, var->size(), res.rows()) = M_i;
    }

    //==========================================================
    //==========================================================
    // Get covariance of the involved terms
    Eigen::MatrixXd P_small = StateHelper::get_marginal_covariance(state, H_order);

    // Residual covariance S = H*Cov*H' + R
    Eigen::MatrixXd S(R.rows(), R.rows());
    S.triangularView<Eigen::Upper>() = H * P_small * H.transpose();
    S.triangularView<Eigen::Upper>() += R;
    //Eigen::MatrixXd S = H * P_small * H.transpose() + R;

    // Invert our S (should we use a more stable method here??)
    Eigen::MatrixXd Sinv = Eigen::MatrixXd::Identity(R.rows(), R.rows());
    S.selfadjointView<Eigen::Upper>().llt().solveInPlace(Sinv);
    Eigen::MatrixXd K = M_a * Sinv.selfadjointView<Eigen::Upper>();
    //Eigen::MatrixXd K = M_a * S.inverse();

    // Update Covariance
    Cov.triangularView<Eigen::Upper>() -= K * M_a.transpose();
    Cov = Cov.selfadjointView<Eigen::Upper>();
    //Cov -= K * M_a.transpose();
    //Cov = 0.5*(Cov+Cov.transpose());

    // We should check if we are not positive semi-definitate (i.e. negative diagionals is not s.p.d)
    Eigen::VectorXd diags = Cov.diagonal();
    for(int i=0; i<diags.rows(); i++) {
        assert(diags(i)>=0.0);
    }

    // Calculate our delta and pass it to update all our state variables
    //cout << "dx = " << endl << (K*res).transpose() << endl;
    state->update(K * res);

}



Eigen::MatrixXd StateHelper::get_marginal_covariance(State *state, const std::vector<Type *> &small_variables) {

    // Calculate the marginal covariance size we need to make our matrix
    int cov_size = 0;
    for (size_t i = 0; i < small_variables.size(); i++) {
        cov_size += small_variables[i]->size();
    }

    // Construct our return covariance
    Eigen::MatrixXd Small_cov = Eigen::MatrixXd::Zero(cov_size, cov_size);

    // For each variable, lets copy over all other variable cross terms
    // Note: this copies over itself to when i_index=k_index
    int i_index = 0;
    for (size_t i = 0; i < small_variables.size(); i++) {
        int k_index = 0;
        for (size_t k = 0; k < small_variables.size(); k++) {
            Small_cov.block(i_index, k_index, small_variables[i]->size(), small_variables[k]->size()) =
                    state->_Cov.block(small_variables[i]->id(), small_variables[k]->id(), small_variables[i]->size(), small_variables[k]->size());
            k_index += small_variables[k]->size();
        }
        i_index += small_variables[i]->size();
    }

    // Return the covariance
    //Small_cov = 0.5*(Small_cov+Small_cov.transpose());
    return Small_cov;
}




void StateHelper::marginalize(State *state, Type *marg) {

    // Check if the current state has the element we want to marginalize
    if (std::find(state->variables().begin(), state->variables().end(), marg) == state->variables().end()) {
        std::cerr << "CovManager::marginalize() - Called on variable that is not in the state" << std::endl;
        std::cerr << "CovManager::marginalize() - Marginalization, does NOT work on sub-variables yet..." << std::endl;
        std::exit(EXIT_FAILURE);
    }

    //Generic covariance has this form for x_1, x_m, x_2. If we want to remove x_m:
    //
    //  P_(x_1,x_1) P(x_1,x_m) P(x_1,x_2)
    //  P_(x_m,x_1) P(x_m,x_m) P(x_m,x_2)
    //  P_(x_2,x_1) P(x_2,x_m) P(x_2,x_2)
    //
    //  to
    //
    //  P_(x_1,x_1) P(x_1,x_2)
    //  P_(x_2,x_1) P(x_2,x_2)
    //
    // i.e. x_1 goes from 0 to marg_id, x_2 goes from marg_id+marg_size to Cov.rows() in the original covariance

    int marg_size = marg->size();
    int marg_id = marg->id();

    Eigen::MatrixXd Cov_new(state->n_vars() - marg_size, state->n_vars() - marg_size);

    int x2_size = (int)state->n_vars() - marg_id - marg_size;

    //P_(x_1,x_1)
    Cov_new.block(0, 0, marg_id, marg_id) = state->Cov().block(0, 0, marg_id, marg_id);

    //P_(x_1,x_2)
    Cov_new.block(0, marg_id, marg_id, x2_size) = state->Cov().block(0, marg_id + marg_size, marg_id, x2_size);

    //P_(x_2,x_1)
    Cov_new.block(marg_id, 0, x2_size, marg_id) = Cov_new.block(0, marg_id, marg_id, x2_size).transpose();

    //P(x_2,x_2)
    Cov_new.block(marg_id, marg_id, x2_size, x2_size) = state->Cov().block(marg_id + marg_size, marg_id + marg_size, x2_size, x2_size);


    // Now set new covariance
    state->Cov() = Cov_new;
    //state->Cov() = 0.5*(Cov_new+Cov_new.transpose());
    assert(state->Cov().rows() == Cov_new.rows());

    // Now we keep the remaining variables and update their ordering
    // Note: DOES NOT SUPPORT MARGINALIZING SUBVARIABLES YET!!!!!!!
    std::vector<Type *> remaining_variables;
    for (size_t i = 0; i < state->variables().size(); i++) {
        //Only keep non-marginal states
        if (state->variables(i) != marg) {
            if (state->variables(i)->id() > marg_id) {
                //If the variable is "beyond" the marginal one in ordering, need to "move it forward"
                state->variables(i)->set_local_id(state->variables(i)->id() - marg_size);
            }
            remaining_variables.push_back(state->variables(i));
        }
    }

    // Delete the old state variable to free up its memory
    delete marg;

    // Now set variables as the remaining ones
    state->variables() = remaining_variables;

}

// 将状态变量中Type类型的变量复制一份放到放在状态变量的尾部并同时维护协方差矩阵
Type* StateHelper::clone(State *state, Type *variable_to_clone) {

    //Get total size of new cloned variables, and the old covariance size
    int total_size = variable_to_clone->size();               // 新变量的总长度
    int old_size = (int) state->n_vars();                     // 原来变量的总长度
    int new_loc = (int) state->n_vars();                      // 新变量的位置

    // Resize both our covariance to the new size
    state->Cov().conservativeResizeLike(Eigen::MatrixXd::Zero(state->n_vars() + total_size, state->n_vars() + total_size)); // resize 协方差矩阵，增加新的变量维度

    // What is the new state, and variable we inserted
    const std::vector<Type*> new_variables = state->variables();                  // 新的状态变量
    Type *new_clone = nullptr;                                                    // 克隆的状态变量

    // Loop through all variables, and find the variable that we are going to clone
    for (size_t k = 0; k < state->variables().size(); k++) {                      // 从原来的状态向量中找到需要克隆的状态变量

        // Skip this if it is not the same
        Type *type_check = state->variables(k)->check_if_same_variable(variable_to_clone); // 遍历所有的状态变量，找到相同的状态变量，type_check已经指到了当前需要克隆的状态变量
        if (type_check == nullptr)
            continue;

        // So we will clone this one
        int old_loc = type_check->id();                                                    // 找到状态变量后进行克隆

        // Copy the covariance elements                                                    // 复制协方差矩阵
        state->Cov().block(new_loc, new_loc, total_size, total_size) = state->Cov().block(old_loc, old_loc, total_size, total_size);
        state->Cov().block(0, new_loc, old_size, total_size) = state->Cov().block(0, old_loc, old_size, total_size);
        state->Cov().block(new_loc, 0, total_size, old_size) = state->Cov().block(old_loc, 0, total_size, old_size);

        // Create clone from the type being cloned
        new_clone = type_check->clone();                                        // 返回新的clone
        new_clone->set_local_id(new_loc);                                       // 设置在协方差矩阵中新的id

        // Add to variable list
        state->insert_variable(new_clone);                                      // 将新的 clone 插入到状态变量中
        break;

    }

    // Check if the current state has this variable
    if (new_clone == nullptr) {
        std::cerr << "CovManager::clone() - Called on variable is not in the state" << std::endl;
        std::cerr << "CovManager::clone() - Ensure that the variable specified is a variable, or sub-variable.." << std::endl;
        std::exit(EXIT_FAILURE);
    }

    return new_clone;

}




bool StateHelper::initialize(State *state, Type *new_variable, const std::vector<Type *> &H_order, Eigen::MatrixXd &H_R,
                             Eigen::MatrixXd &H_L, Eigen::MatrixXd &R, Eigen::VectorXd &res, double chi_2_mult) {

    // Check that this new variable is not already initialized
    if (std::find(state->variables().begin(), state->variables().end(), new_variable) != state->variables().end()) {
        std::cerr << "CovManager::initialize() - Called on variable that is already in the state" << std::endl;
        std::cerr << "CovManager::initialize() - Found this variable at " << new_variable->id() << " in covariance" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    // Check that we have isotropic noise (i.e. is diagonal and all the same value)
    // TODO: can we simplify this so it doesn't take as much time?
    assert(R.rows()==R.cols());
    assert(R.rows()>0);
    for(int r=0; r<R.rows(); r++) {
        for(int c=0; c<R.cols(); c++) {
            if(r==c && R(0,0) != R(r,c)) {
                std::cerr << "CovManager::initialize() - Your noise is not isotropic!" << std::endl;
                std::cerr << "CovManager::initialize() - Found a value of " << R(r,c) << " vs " << R(0,0)  << " at row " << r << " column " << c << std::endl;
                std::exit(EXIT_FAILURE);
            } else if(r!=c && R(r,c) != 0.0) {
                std::cerr << "CovManager::initialize() - Your noise is not diagonal!" << std::endl;
                std::cerr << "CovManager::initialize() - Found a value of " << R(r,c) << " at row " << r << " column " << c << std::endl;
                std::exit(EXIT_FAILURE);
            }
        }
    }

    //==========================================================
    //==========================================================
    // First we perform QR givens to seperate the system
    // The top will be a system that depends on the new state, while the bottom does not
    size_t new_var_size = new_variable->size();
    assert((int)new_var_size == H_L.cols());

    Eigen::JacobiRotation<double> tempHo_GR;
    for (int n = 0; n < H_L.cols(); ++n) {
        for (int m = (int) H_L.rows() - 1; m > n; m--) {
            // Givens matrix G
            tempHo_GR.makeGivens(H_L(m - 1, n), H_L(m, n));
            // Multiply G to the corresponding lines (m-1,m) in each matrix
            // Note: we only apply G to the nonzero cols [n:Ho.cols()-n-1], while
            //       it is equivalent to applying G to the entire cols [0:Ho.cols()-1].
            (H_L.block(m - 1, n, 2, H_L.cols() - n)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
            (res.block(m - 1, 0, 2, 1)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
            (H_R.block(m - 1, 0, 2, H_R.cols())).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
        }
    }

    // Separate into initializing and updating portions
    // 1. Invertible initializing system
    Eigen::MatrixXd Hxinit = H_R.block(0, 0, new_var_size, H_R.cols());
    Eigen::MatrixXd H_finit = H_L.block(0, 0, new_var_size, new_var_size);
    Eigen::VectorXd resinit = res.block(0, 0, new_var_size, 1);
    Eigen::MatrixXd Rinit = R.block(0, 0, new_var_size, new_var_size);

    // 2. Nullspace projected updating system
    Eigen::MatrixXd Hup = H_R.block(new_var_size, 0, H_R.rows() - new_var_size, H_R.cols());
    Eigen::VectorXd resup = res.block(new_var_size, 0, res.rows() - new_var_size, 1);
    Eigen::MatrixXd Rup = R.block(new_var_size, new_var_size, R.rows() - new_var_size, R.rows() - new_var_size);

    //==========================================================
    //==========================================================

    // Do mahalanobis distance testing
    Eigen::MatrixXd P_up = get_marginal_covariance(state, H_order);
    assert(Rup.rows() == Hup.rows());
    assert(Hup.cols() == P_up.cols());
    Eigen::MatrixXd S = Hup*P_up*Hup.transpose()+Rup;
    double chi2 = resup.dot(S.llt().solve(resup));

    // Get what our threshold should be
    boost::math::chi_squared chi_squared_dist(res.rows());
    double chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
    if (chi2 > chi_2_mult*chi2_check){
        return false;
    }

    //==========================================================
    //==========================================================
    // Finally, initialize it in our state
    StateHelper::initialize_invertible(state, new_variable, H_order, Hxinit, H_finit, Rinit, resinit);

    // Update with updating portion
    if (Hup.rows() > 0) {
        StateHelper::EKFUpdate(state, H_order, Hup, resup, Rup);
    }
    return true;

}


void StateHelper::initialize_invertible(State *state, Type *new_variable, const std::vector<Type *> &H_order, const Eigen::MatrixXd &H_R,
                                        const Eigen::MatrixXd &H_L, const Eigen::MatrixXd &R, const Eigen::VectorXd &res) {

    // Check that this new variable is not already initialized
    if (std::find(state->variables().begin(), state->variables().end(), new_variable) != state->variables().end()) {
        std::cerr << "CovManager::initialize_invertible() - Called on variable that is already in the state" << std::endl;
        std::cerr << "CovManager::initialize_invertible() - Found this variable at " << new_variable->id() << " in covariance" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    // Check that we have isotropic noise (i.e. is diagonal and all the same value)
    // TODO: can we simplify this so it doesn't take as much time?
    assert(R.rows()==R.cols());
    assert(R.rows()>0);
    for(int r=0; r<R.rows(); r++) {
        for(int c=0; c<R.cols(); c++) {
            if(r==c && R(0,0) != R(r,c)) {
                std::cerr << "CovManager::initialize_invertible() - Your noise is not isotropic!" << std::endl;
                std::cerr << "CovManager::initialize_invertible() - Found a value of " << R(r,c) << " verses value of " << R(0,0) << std::endl;
                std::exit(EXIT_FAILURE);
            } else if(r!=c && R(r,c) != 0.0) {
                std::cerr << "CovManager::initialize_invertible() - Your noise is not diagonal!" << std::endl;
                std::cerr << "CovManager::initialize_invertible() - Found a value of " << R(r,c) << " at row " << r << " column " << c << std::endl;
                std::exit(EXIT_FAILURE);
            }
        }
    }

    //==========================================================
    //==========================================================
    // Part of the Kalman Gain K = (P*H^T)*S^{-1} = M*S^{-1}
    assert(res.rows() == R.rows());
    assert(H_L.rows() == res.rows());
    assert(H_L.rows() == H_R.rows());
    Eigen::MatrixXd M_a = Eigen::MatrixXd::Zero(state->n_vars(), res.rows());

    // Get the location in small jacobian for each measuring variable
    int current_it = 0;
    std::vector<int> H_id;
    for (Type *meas_var: H_order) {
        H_id.push_back(current_it);
        current_it += meas_var->size();
    }

    //==========================================================
    //==========================================================
    // For each active variable find its M = P*H^T
    for (Type *var: state->variables()) {
        // Sum up effect of each subjacobian= K_i= \sum_m (P_im Hm^T)
        Eigen::MatrixXd M_i = Eigen::MatrixXd::Zero(var->size(), res.rows());
        for (size_t i = 0; i < H_order.size(); i++) {
            Type *meas_var = H_order[i];
            M_i += state->Cov().block(var->id(), meas_var->id(), var->size(), meas_var->size()) *
                   H_R.block(0, H_id[i], H_R.rows(), meas_var->size()).transpose();
        }
        M_a.block(var->id(), 0, var->size(), res.rows()) = M_i;
    }


    //==========================================================
    //==========================================================
    // Get covariance of this small jacobian
    Eigen::MatrixXd P_small = StateHelper::get_marginal_covariance(state, H_order);

    // M = H_R*Cov*H_R' + R
    Eigen::MatrixXd M(H_R.rows(), H_R.rows());
    M.triangularView<Eigen::Upper>() = H_R * P_small * H_R.transpose();
    M.triangularView<Eigen::Upper>() += R;

    // Covariance of the variable/landmark that will be initialized
    assert(H_L.rows()==H_L.cols());
    assert(H_L.rows() == new_variable->size());
    Eigen::MatrixXd H_Linv = H_L.inverse();
    Eigen::MatrixXd P_LL = H_Linv * M.selfadjointView<Eigen::Upper>() * H_Linv.transpose();

    // Augment the covariance matrix
    size_t oldSize = state->n_vars();
    state->Cov().conservativeResizeLike(Eigen::MatrixXd::Zero(state->n_vars() + new_variable->size(),state->n_vars() + new_variable->size()));
    state->Cov().block(0, oldSize, oldSize, new_variable->size()).noalias() = -M_a * H_Linv.transpose();
    state->Cov().block(oldSize, 0, new_variable->size(), oldSize) = state->Cov().block(0, oldSize, oldSize, new_variable->size()).transpose();
    state->Cov().block(oldSize, oldSize, new_variable->size(), new_variable->size()) = P_LL;

    // Update the variable that will be initialized (invertible systems can only update the new variable).
    // However this update should be almost zero if we already used a conditional Gauss-Newton to solve for the initial estimate
    new_variable->update(H_Linv * res);

    // Now collect results, and add it to the state variables
    new_variable->set_local_id((int)(state->n_vars()-new_variable->size()));
    state->insert_variable(new_variable);
    //std::cout << new_variable->id() <<  " init dx = " << (H_Linv * res).transpose() << std::endl;

}




void StateHelper::augment_clone(State *state, Eigen::Matrix<double, 3, 1> last_w) {

    // Get our state and covariance
    auto imu = state->imu();                                                               // 获得imu的状态和协方差
    auto &Cov = state->Cov();

    // Call on our marginalizer to clone, it will add it to our vector of types
    // NOTE: this will clone the clone pose to the END of the covariance...
    Type *posetemp = StateHelper::clone(state, imu->pose());                               // 克隆IMU对应的位姿(q,p)到状态向量的最后，并克隆协方差矩阵，PoseJPL(只克隆IMU的q和p)

    // Cast to a JPL pose type
    PoseJPL *pose = dynamic_cast<PoseJPL*>(posetemp);

    // Check that it was a valid cast
    if (pose == nullptr) {
        //ROS_ERROR("INVALID OBJECT RETURNED FROM MARGINALIZER, EXITING!#!@#!@#");
        exit(EXIT_FAILURE);
    }

    // Append the new clone to our clone vector
    state->insert_clone(state->timestamp(), pose);                                        // 将新的IMU位姿估计为当前Image的时间戳和位姿，插入到clone状态map中

    // If we are doing time calibration, then our clones are a function of the time offset
    // Logic is based on Mingyang Li and Anastasios I. Mourikis paper:
    // http://journals.sagepub.com/doi/pdf/10.1177/0278364913515286
    if (state->options().do_calib_camera_timeoffset) {                                    // 如果考虑时间同步问题，需要往前传播一下，依据时间同步计算协方差
        // Jacobian to augment by
        Eigen::Matrix<double, 6, 1> dnc_dt = Eigen::MatrixXd::Zero(6, 1);
        dnc_dt.block(0, 0, 3, 1) = last_w;
        dnc_dt.block(3, 0, 3, 1) = imu->vel();
        // Augment covariance with time offset Jacobian
        Cov.block(0, pose->id(), Cov.rows(), 6) += Cov.block(0, state->calib_dt_CAMtoIMU()->id(), Cov.rows(), 1) * dnc_dt.transpose();
        Cov.block(pose->id(), 0, 6, Cov.rows()) += dnc_dt * Cov.block(state->calib_dt_CAMtoIMU()->id(), 0, 1, Cov.rows());
        Cov.block(pose->id(), pose->id(), 6, 6) += dnc_dt * Cov(state->calib_dt_CAMtoIMU()->id(),
                                                                state->calib_dt_CAMtoIMU()->id()) * dnc_dt.transpose();
    }

}


