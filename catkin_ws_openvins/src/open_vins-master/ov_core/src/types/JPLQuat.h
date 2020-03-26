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
#ifndef OV_CORE_TYPE_JPLQUAT_H
#define OV_CORE_TYPE_JPLQUAT_H


#include "Type.h"
#include "utils/quat_ops.h"


namespace ov_core {


    /**
     * @brief Derived Type class that implements JPL quaternion
     *
     * This quaternion uses a left-multiplicative error state and follows the "JPL convention".
     * Please checkout our utility functions in the quat_ops.h file.
     * We recommend that people new quaternions check out the following resources:
     * - http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf
     * - ftp://naif.jpl.nasa.gov/pub/naif/misc/Quaternion_White_Paper/Quaternions_White_Paper.pdf
     */

    // JPL 四元数类型的变量参数，表示旋转，自由度为3，实际变量长度为4
    class JPLQuat : public Type {

    public:
        // JPL 四元数，实际内部存储为4*1的矩阵，虚部为1
        JPLQuat() : Type(3) {
            Eigen::Matrix<double, 4, 1> q0;
            q0.setZero();
            q0(3) = 1.0;
            set_value(q0);
            set_fej(q0);
        }

        ~JPLQuat() {}

        /**
         * @brief Implements update operation by left-multiplying the current
         * quaternion with a quaternion built from a small axis-angle perturbation.
         *
         * @f[
         * \bar{q}=norm\Big(\begin{bmatrix} 0.5*\mathbf{\theta_{dx}} \\ 1 \end{bmatrix}\Big) \hat{\bar{q}}
         * @f]
         *
         * @param dx Axis-angle representation of the perturbing quaternion  // 3 degree perturbing quaternion Axis-angle left multiply
         */
        // JPL 四元数的增量更新，采用增量的左乘策略，增量dx表示角度自由度的增量
        void update(const Eigen::VectorXd dx) override {

            assert(dx.rows() == _size);

            //Build perturbing quaternion
            Eigen::Matrix<double, 4, 1> dq;
            dq << .5 * dx, 1.0;
            dq = quatnorm(dq);

            //Update estimate and recompute R
            set_value(quat_multiply(dq, _value));

        }

        /**
        * @brief Sets the value of the estimate and recomputes the internal rotation matrix
        * @param new_value New value for the quaternion estimate
        */
        // 设置四元数的值，同时也会设置旋转矩阵的值，四元数和旋转矩阵直接关联
        void set_value(const Eigen::MatrixXd new_value) override {

            assert(new_value.rows() == 4);
            assert(new_value.cols() == 1);

            _value = new_value;

            //compute associated rotation
            _R = quat_2_Rot(new_value);
        }

        // 返回JPL四元数的一个克隆，包括四元数的值和fej的值
        Type *clone() override {
            Type *Clone = new JPLQuat();
            Clone->set_value(value());
            Clone->set_fej(fej());
            return Clone;
        }

        /**
        * @brief Sets the fej value and recomputes the fej rotation matrix
        * @param new_value New value for the quaternion estimate
        */
        // 设置四元式的fej,实维维度 4*1
        void set_fej(const Eigen::MatrixXd new_value) override {

            assert(new_value.rows() == 4);
            assert(new_value.cols() == 1);

            _fej = new_value;

            //compute associated rotation
            _Rfej = quat_2_Rot(new_value);
        }

        /// Rotation access  获取JPL四数数的旋转矩阵
        Eigen::Matrix<double, 3, 3> Rot() const {
            return _R;
        }

        /// FEJ Rotation access      得得JPL四元数fej的旋转矩阵
        Eigen::Matrix<double, 3, 3> Rot_fej() const {
            return _Rfej;
        }

    protected:

        // Stores the rotation
        Eigen::Matrix<double, 3, 3> _R;

        // Stores the first-estimate rotation
        Eigen::Matrix<double, 3, 3> _Rfej;

    };


}

#endif //OV_CORE_TYPE_JPLQUAT_H
