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
#ifndef OPEN_VINS_FEATUREREPRESENTATION_H
#define OPEN_VINS_FEATUREREPRESENTATION_H



namespace ov_core {

    /**
     * @brief Class has useful feature representation types
     */
     // 特征点的表示方式类
    class FeatureRepresentation
    {

    public:

        /**
         * @brief What feature representation our state can use
         */
         // 5种表示方式
        enum Representation {
            GLOBAL_3D,                    // 全局坐标xyz
            GLOBAL_FULL_INVERSE_DEPTH,    // 全局极坐标逆深度
            ANCHORED_3D,                  // 局部坐标xyz
            ANCHORED_FULL_INVERSE_DEPTH,  // 局部极坐标逆深度
            ANCHORED_MSCKF_INVERSE_DEPTH  // 局部msckf逆深度
        };


        /**
         * @brief Helper function that checks if the passed feature representation is a relative or global
         * @param feat_representation Representation we want to check
         * @return True if it is a relative representation
         */
        // 判断是否是局部坐标系
        static inline bool is_relative_representation(Representation feat_representation) {
            return (feat_representation == Representation::ANCHORED_3D ||
                    feat_representation == Representation::ANCHORED_FULL_INVERSE_DEPTH ||
                    feat_representation == Representation::ANCHORED_MSCKF_INVERSE_DEPTH);
        }

    private:

        /**
         * All function in this class should be static.
         * Thus an instance of this class cannot be created.
         */
        FeatureRepresentation() {};

    };



}


#endif //OPEN_VINS_FEATUREREPRESENTATION_H