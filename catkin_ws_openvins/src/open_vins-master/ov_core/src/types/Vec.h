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
#ifndef OV_CORE_TYPE_VEC_H
#define OV_CORE_TYPE_VEC_H


#include "Type.h"


namespace ov_core {

    /**
     * @brief Derived Type class that implements vector variables
     */

    // vector 类型的变量
    class Vec : public Type {

    public:

        /**
         * @brief Default constructor for Vec
         * @param dim Size of the vector (will be same as error state)
         */
         // Vec 的默认构造函数，输入为vec 的维度，利用vectorXd来构造
        Vec(int dim) : Type(dim) {
            _value = Eigen::VectorXd::Zero(dim);
            _fej = Eigen::VectorXd::Zero(dim);
        }

        ~Vec() {}

        /**
         * @brief Implements the update operation through standard vector addition
         * @param dx Additive error state correction
         */
        // 增量更新，直在在vec变量上进行增量增加
        void update(const Eigen::VectorXd dx) override {
            assert(dx.rows() == _size);
            set_value(_value + dx);
        }

        /**
         * @brief Performs all the cloning
         */
         // vector类型的clone，新建一个vector，将value和fej的值制制过去并返回
        Type *clone() override {
            Type *Clone = new Vec(_size);
            Clone->set_value(value());
            Clone->set_fej(fej());
            return Clone;
        }


    };

}

#endif //OV_CORE_TYPE_VEC_H