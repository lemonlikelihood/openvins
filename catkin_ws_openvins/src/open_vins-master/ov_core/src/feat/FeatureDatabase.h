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
#ifndef OV_CORE_FEATURE_DATABASE_H
#define OV_CORE_FEATURE_DATABASE_H


#include <vector>
#include <mutex>
#include <Eigen/Eigen>

#include "Feature.h"


namespace ov_core {

    /**
     * @brief Database containing features we are currently tracking.
     *
     * Each visual tracker has this database in it and it contains all features that we are tracking.
     * The trackers will insert information into this database when they get new measurements from doing tracking.
     * A user would then query this database for features that can be used for update and remove them after they have been processed.
     *
     *
     * @m_class{m-note m-warning}
     *
     * @par A Note on Multi-Threading Support
     * There is some support for asynchronous multi-threaded access.
     * Since each feature is a pointer just directly returning and using them is not thread safe.
     * Thus, to be thread safe, use the "remove" flag for each function which will remove it from this feature database.
     * This prevents the trackers from adding new measurements and editing the feature information.
     * For example, if you are asynchronous tracking cameras and you chose to update the state, then remove all features you will use in update.
     * The feature trackers will continue to add features while you update, whose measurements can be used in the next update step!
     *
     */
    class FeatureDatabase {

    public:

        /**
         * @brief Default constructor
         */
        FeatureDatabase() {
            features_idlookup = std::unordered_map<size_t, Feature *>();    // Feat_id , Feature *
        }


        /**
         * @brief Get a specified feature
         * @param id What feature we want to get
         * @param remove Set to true if you want to remove the feature from the database (you will need to handle the freeing of memory)
         * @return Either a feature object, or null if it is not in the database.
         */
         // 按特征点id来查找
        Feature *get_feature(size_t id, bool remove=false) {
            std::unique_lock<std::mutex> lck(mtx);
            if (features_idlookup.find(id) != features_idlookup.end()) {
                Feature* temp = features_idlookup[id];
                if(remove) features_idlookup.erase(id);
                return temp;
            } else {
                return nullptr;
            }
        }


        /**
         * @brief Update a feature object
         * @param id ID of the feature we will update
         * @param timestamp time that this measurement occured at
         * @param cam_id which camera this measurement was from
         * @param u raw u coordinate
         * @param v raw v coordinate
         * @param u_n undistorted/normalized u coordinate
         * @param v_n undistorted/normalized v coordinate
         *
         * This will update a given feature based on the passed ID it has.
         * It will create a new feature, if it is an ID that we have not seen before.
         */
        // 更新特征点坐标（若是新的增加，若是已有则在容器末尾添加）
        void update_feature(size_t id, double timestamp, size_t cam_id,
                            float u, float v, float u_n, float v_n) {

            // Find this feature using the ID lookup
            std::unique_lock<std::mutex> lck(mtx);
            if (features_idlookup.find(id) != features_idlookup.end()) {
                // Get our feature
                Feature *feat = features_idlookup[id];
                // Append this new information to it!   放到对应的相机时间戳里面
                feat->uvs[cam_id].emplace_back(Eigen::Vector2f(u, v));
                feat->uvs_norm[cam_id].emplace_back(Eigen::Vector2f(u_n, v_n));
                feat->timestamps[cam_id].emplace_back(timestamp);
                return;
            }

            // Debug info
            //ROS_INFO("featdb - adding new feature %d",(int)id);

            // Else we have not found the feature, so lets make it be a new one!
            Feature *feat = new Feature();
            feat->featid = id;
            feat->uvs[cam_id].emplace_back(Eigen::Vector2f(u, v));
            feat->uvs_norm[cam_id].emplace_back(Eigen::Vector2f(u_n, v_n));
            feat->timestamps[cam_id].emplace_back(timestamp);

            // Append this new feature into our database
            features_idlookup.insert({id, feat});
        }


        /**
         * @brief Get features that do not have newer measurement then the specified time.
         *
         * This function will return all features that do not a measurement at a time greater than the specified time.
         * For example this could be used to get features that have not been successfully tracked into the newest frame.
         * All features returned will not have any measurements occurring at a time greater then the specified.
         */

        // 返回跟踪时长小于当前时间戳的track，即该特征点的结束跟踪时间戳小于当前时间戳，返回在timestamp之后已经丢失的特征点
        std::vector<Feature *> features_not_containing_newer(double timestamp, bool remove=false) {

            // Our vector of features that do not have measurements after the specified time
            std::vector<Feature *> feats_old;

            // Now lets loop through all features, and just make sure they are not old
            std::unique_lock<std::mutex> lck(mtx);
            for (auto it = features_idlookup.begin(); it != features_idlookup.end();) {
                // Loop through each camera
                bool has_newer_measurement = false;
                for (auto const &pair : (*it).second->timestamps) {
                    // pair{size_t camera_id,std::vector<double> t}，只要其中一个相机的时间戳大于timestamp就行，跳出相机循环
                    // If we have a measurement greater-than or equal to the specified, this measurement is find
                    if (!pair.second.empty() && pair.second.at(pair.second.size() - 1) >= timestamp) {
                        has_newer_measurement = true;
                        break;
                    }
                }
                // If it is not being actively tracked, then it is old
                if (!has_newer_measurement) {
                    feats_old.push_back((*it).second);
                    if(remove) features_idlookup.erase(it++);
                    else it++;
                } else {
                    it++;
                }
            }

            // Debugging
            //std::cout << "feature db size = " << features_idlookup.size() << std::endl;

            // Return the old features
            return feats_old;

        }


        /**
         * @brief Get features that has measurements older then the specified time.
         *
         * This will collect all features that have measurements occurring before the specified timestamp.
         * For example, we would want to remove all features older then the last clone/state in our sliding window.
         */

        // 返回跟踪时长久于当前时间戳的track,即该特征点的起始跟踪时间戳小于当前时间戳
        std::vector<Feature *> features_containing_older(double timestamp, bool remove=false) {

            // Our vector of old features
            std::vector<Feature *> feats_old;

            // Now lets loop through all features, and just make sure they are not old
            std::unique_lock<std::mutex> lck(mtx);
            for (auto it = features_idlookup.begin(); it != features_idlookup.end();) {
                // Loop through each camera
                bool found_containing_older = false;
                for (auto const &pair : (*it).second->timestamps) {
                    if (!pair.second.empty() && pair.second.at(0) < timestamp) {
                        found_containing_older = true;
                        break;
                    }
                }
                // If it has an older timestamp, then add it
                if(found_containing_older) {
                    feats_old.push_back((*it).second);
                    if(remove) features_idlookup.erase(it++);
                    else it++;
                } else {
                    it++;
                }
            }

            // Debugging
            //std::cout << "feature db size = " << features_idlookup.size() << std::endl;

            // Return the old features
            return feats_old;

        }

        /**
         * @brief Get features that has measurements at the specified time.
         *
         * This function will return all features that have the specified time in them.
         * This would be used to get all features that occurred at a specific clone/state.
         */

        // 返回track时间包含当前时间戳的track，返回在timestamp跟踪的特征点，相当于 = timestamp（用于Marg掉滑窗内最老的MSCKF状态对应时间戳的那些特征点）
        std::vector<Feature *> features_containing(double timestamp, bool remove=false) {

            // Our vector of old features
            std::vector<Feature *> feats_has_timestamp;

            // Now lets loop through all features, and just make sure they are not
            std::unique_lock<std::mutex> lck(mtx);
            for (auto it = features_idlookup.begin(); it != features_idlookup.end();) {
                // Boolean if it has the timestamp
                bool has_timestamp = false;
                for (auto const &pair : (*it).second->timestamps) {
                    // Loop through all timestamps, and see if it has it
                    for (auto &timefeat : pair.second) {
                        if (timefeat == timestamp) {
                            has_timestamp = true;
                            break;
                        }
                    }
                    // Break out if we found a single timestamp that is equal to the specified time
                    if (has_timestamp) {
                        break;
                    }
                }
                // Remove this feature if it contains the specified timestamp
                if (has_timestamp) {
                    feats_has_timestamp.push_back((*it).second);
                    if(remove) features_idlookup.erase(it++);
                    else it++;
                } else {
                    it++;
                }
            }

            // Debugging
            //std::cout << "feature db size = " << features_idlookup.size() << std::endl;
            //std::cout << "return vector = " << feats_has_timestamp.size() << std::endl;

            // Return the features
            return feats_has_timestamp;

        }

        /**
         * @brief This function will delete all features that have been used up.
         *
         * If a feature was unable to be used, it will still remain since it will not have a delete flag set
         */

        // 删除那些to_delete被设置为true的feature，仅有在更新完MSCKF的之后才执行，删除标志位to_delete=true的点容器features_idlookup
        void cleanup() {
            // Debug
            //int sizebefore = (int)features_idlookup.size();
            // Loop through all features
            std::unique_lock<std::mutex> lck(mtx);
            for (auto it = features_idlookup.begin(); it != features_idlookup.end();) {
                // If delete flag is set, then delete it
                if ((*it).second->to_delete) {
                    delete (*it).second;
                    features_idlookup.erase(it++);
                } else {
                    it++;
                }
            }
            // Debug
            //std::cout << "feat db = " << sizebefore << " -> " << (int)features_idlookup.size() << std::endl;
        }


        /**
         * @brief Returns the size of the feature database
         */
        size_t size() {
            std::unique_lock<std::mutex> lck(mtx);
            return features_idlookup.size();
        }


        /**
         * @brief Returns the internal data (should not normally be used)
         */
        std::unordered_map<size_t, Feature *> get_internal_data() {
            std::unique_lock<std::mutex> lck(mtx);
            return features_idlookup;
        }

    protected:

        /// Mutex lock for our map
        std::mutex mtx;

        /// Our lookup array that allow use to query based on ID
        ///
        std::unordered_map<size_t, Feature *> features_idlookup;  /// size_t featid , Feature *


    };


}

#endif /* OV_CORE_FEATURE_DATABASE_H */