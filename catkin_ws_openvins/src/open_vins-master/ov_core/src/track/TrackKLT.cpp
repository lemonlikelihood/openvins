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
#include "TrackKLT.h"


using namespace ov_core;

// 接受Image数据，完成track
void TrackKLT::feed_monocular(double timestamp, cv::Mat &img, size_t cam_id) {

    // Start timing
    rT1 =  boost::posix_time::microsec_clock::local_time();

    // Lock this data feed for this camera
    std::unique_lock<std::mutex> lck(mtx_feeds.at(cam_id));

    // Histogram equalize
    cv::equalizeHist(img, img);                                      // 1. 对收到的图片首先做一个直方图均衡化

    // Extract the new image pyramid
    std::vector<cv::Mat> imgpyr;                                     // 2. 对均衡化后的图像提取金字塔（按传入的窗口大小和金字塔层数来提取）
    cv::buildOpticalFlowPyramid(img, imgpyr, win_size, pyr_levels);
    rT2 =  boost::posix_time::microsec_clock::local_time();

    // If we didn't have any successful tracks last time, just extract this time
    // This also handles, the tracking initalization on the first call to this extractor
    if(pts_last[cam_id].empty()) {                                   // 3. 上一帧的关键点为空，直接提取，走检测初始化流程
        // Detect new features
        perform_detection_monocular(imgpyr, pts_last[cam_id], ids_last[cam_id]);
        // Save the current image and pyramid
        img_last[cam_id] = img.clone();
        img_pyramid_last[cam_id] = imgpyr;
        return;
    }

    // First we should make that the last images have enough features so we can do KLT          // 4. 直接对原始图像提取特征点
    // This will "top-off" our number of tracks so always have a constant number
    perform_detection_monocular(img_pyramid_last[cam_id], pts_last[cam_id], ids_last[cam_id]);  // 5. 先对上一帧提取新的特征点，再进行track
    rT3 =  boost::posix_time::microsec_clock::local_time();

    //===================================================================================
    //===================================================================================

    // Debug
    //ROS_INFO("current points = %d,%d",(int)pts_left_last.size(),(int)pts_right_last.size());

    // Our return success masks, and predicted new features
    std::vector<uchar> mask_ll;
    std::vector<cv::KeyPoint> pts_left_new = pts_last[cam_id];

    // Lets track temporally         // 上一帧金字塔  // 当前帧金字塔 // 上一帧关键点(原图) // 当前帧关键点(原图)
    perform_matching(img_pyramid_last[cam_id],imgpyr,pts_last[cam_id],pts_left_new,cam_id,cam_id,mask_ll);   // mask_ll 表明track是否成功
    rT4 =  boost::posix_time::microsec_clock::local_time();

    //===================================================================================
    //===================================================================================

    // If any of our mask is empty, that means we didn't have enough to do ransac, so just return
    if(mask_ll.empty()) {
        img_last[cam_id] = img.clone();
        img_pyramid_last[cam_id] = imgpyr;
        pts_last[cam_id].clear();
        ids_last[cam_id].clear();
        ROS_ERROR("[KLT-EXTRACTOR]: Failed to get enough points to do RANSAC, resetting.....");
        return;
    }

    // Get our "good tracks"               从匹配成功的mask里面挑选出好的特征点
    std::vector<cv::KeyPoint> good_left;
    std::vector<size_t> good_ids_left;

    // Loop through all left points
    for(size_t i=0; i<pts_left_new.size(); i++) {
        // Ensure we do not have any bad KLT tracks (i.e., points are negative)
        if(pts_left_new[i].pt.x < 0 || pts_left_new[i].pt.y < 0)
            continue;
        // If it is a good track, and also tracked from left to right
        if(mask_ll[i]) {
            good_left.push_back(pts_left_new[i]);
            good_ids_left.push_back(ids_last[cam_id][i]);
        }
    }


    //===================================================================================
    //===================================================================================


    // Update our feature database, with theses new observations                    // 更新特征点database中的状态
    for(size_t i=0; i<good_left.size(); i++) {
        cv::Point2f npt_l = undistort_point(good_left.at(i).pt, cam_id);           // 原始图像上去畸变后的特征点
        database->update_feature(good_ids_left.at(i), timestamp, cam_id,
                                 good_left.at(i).pt.x, good_left.at(i).pt.y,
                                 npt_l.x, npt_l.y);
    }

    // Move forward in time
    img_last[cam_id] = img.clone();                                                 // 时间上后移
    img_pyramid_last[cam_id] = imgpyr;
    pts_last[cam_id] = good_left;
    ids_last[cam_id] = good_ids_left;
    rT5 =  boost::posix_time::microsec_clock::local_time();

    // Timing information
    //ROS_INFO("[TIME-KLT]: %.4f seconds for pyramid",(rT2-rT1).total_microseconds() * 1e-6);
    //ROS_INFO("[TIME-KLT]: %.4f seconds for detection",(rT3-rT2).total_microseconds() * 1e-6);
    //ROS_INFO("[TIME-KLT]: %.4f seconds for temporal klt",(rT4-rT3).total_microseconds() * 1e-6);
    //ROS_INFO("[TIME-KLT]: %.4f seconds for feature DB update (%d features)",(rT5-rT4).total_microseconds() * 1e-6, (int)good_left.size());
    //ROS_INFO("[TIME-KLT]: %.4f seconds for total",(rT5-rT1).total_microseconds() * 1e-6);


}


void TrackKLT::feed_stereo(double timestamp, cv::Mat &img_leftin, cv::Mat &img_rightin, size_t cam_id_left, size_t cam_id_right) {

    // Start timing
    rT1 =  boost::posix_time::microsec_clock::local_time();

    // Lock this data feed for this camera
    std::unique_lock<std::mutex> lck1(mtx_feeds.at(cam_id_left));
    std::unique_lock<std::mutex> lck2(mtx_feeds.at(cam_id_right));

    // Histogram equalize
    cv::Mat img_left, img_right;
    boost::thread t_lhe = boost::thread(cv::equalizeHist, boost::cref(img_leftin), boost::ref(img_left));
    boost::thread t_rhe = boost::thread(cv::equalizeHist, boost::cref(img_rightin), boost::ref(img_right));
    t_lhe.join();
    t_rhe.join();

    // Extract image pyramids (boost seems to require us to put all the arguments even if there are defaults....)
    std::vector<cv::Mat> imgpyr_left, imgpyr_right;
    boost::thread t_lp = boost::thread(cv::buildOpticalFlowPyramid, boost::cref(img_left),
                                       boost::ref(imgpyr_left), boost::ref(win_size), boost::ref(pyr_levels), false,
                                       cv::BORDER_REFLECT_101, cv::BORDER_CONSTANT, true);
    boost::thread t_rp = boost::thread(cv::buildOpticalFlowPyramid, boost::cref(img_right),
                                       boost::ref(imgpyr_right), boost::ref(win_size), boost::ref(pyr_levels),
                                       false, cv::BORDER_REFLECT_101, cv::BORDER_CONSTANT, true);
    t_lp.join();
    t_rp.join();
    rT2 =  boost::posix_time::microsec_clock::local_time();

    // If we didn't have any successful tracks last time, just extract this time
    // This also handles, the tracking initalization on the first call to this extractor
    if(pts_last[cam_id_left].empty() || pts_last[cam_id_right].empty()) {
        // Track into the new image
        perform_detection_stereo(imgpyr_left, imgpyr_right, pts_last[cam_id_left], pts_last[cam_id_right], ids_last[cam_id_left], ids_last[cam_id_right]);
        // Save the current image and pyramid
        img_last[cam_id_left] = img_left.clone();
        img_last[cam_id_right] = img_right.clone();
        img_pyramid_last[cam_id_left] = imgpyr_left;
        img_pyramid_last[cam_id_right] = imgpyr_right;
        return;
    }

    // First we should make that the last images have enough features so we can do KLT
    // This will "top-off" our number of tracks so always have a constant number
    perform_detection_stereo(img_pyramid_last[cam_id_left], img_pyramid_last[cam_id_right],
                             pts_last[cam_id_left], pts_last[cam_id_right],
                             ids_last[cam_id_left], ids_last[cam_id_right]);
    rT3 =  boost::posix_time::microsec_clock::local_time();


    //===================================================================================
    //===================================================================================

    // Our return success masks, and predicted new features
    std::vector<uchar> mask_ll, mask_rr;
    std::vector<cv::KeyPoint> pts_left_new = pts_last[cam_id_left];
    std::vector<cv::KeyPoint> pts_right_new = pts_last[cam_id_right];

    // Lets track temporally
    boost::thread t_ll = boost::thread(&TrackKLT::perform_matching, this, boost::cref(img_pyramid_last[cam_id_left]), boost::cref(imgpyr_left),
                                       boost::ref(pts_last[cam_id_left]), boost::ref(pts_left_new), cam_id_left, cam_id_left, boost::ref(mask_ll));
    boost::thread t_rr = boost::thread(&TrackKLT::perform_matching, this, boost::cref(img_pyramid_last[cam_id_right]), boost::cref(imgpyr_right),
                                       boost::ref(pts_last[cam_id_right]), boost::ref(pts_right_new), cam_id_right, cam_id_right, boost::ref(mask_rr));

    // Wait till both threads finish
    t_ll.join();
    t_rr.join();
    rT4 =  boost::posix_time::microsec_clock::local_time();


    //===================================================================================
    //===================================================================================


    // left to right matching
    std::vector<uchar> mask_lr;
    perform_matching(imgpyr_left, imgpyr_right, pts_left_new, pts_right_new, cam_id_left, cam_id_right, mask_lr);
    rT5 =  boost::posix_time::microsec_clock::local_time();


    //===================================================================================
    //===================================================================================

    // If any of our masks are empty, that means we didn't have enough to do ransac, so just return
    if(mask_ll.empty() || mask_rr.empty() || mask_lr.empty()) {
        img_last[cam_id_left] = img_left.clone();
        img_last[cam_id_right] = img_right.clone();
        img_pyramid_last[cam_id_left] = imgpyr_left;
        img_pyramid_last[cam_id_right] = imgpyr_right;
        pts_last[cam_id_left].clear();
        pts_last[cam_id_right].clear();
        ids_last[cam_id_left].clear();
        ids_last[cam_id_right].clear();
        ROS_ERROR("[KLT-EXTRACTOR]: Failed to get enough points to do RANSAC, resetting.....");
        return;
    }

    // Get our "good tracks"
    std::vector<cv::KeyPoint> good_left, good_right;
    std::vector<size_t> good_ids_left, good_ids_right;

    // Loop through all left points
    for(size_t i=0; i<pts_left_new.size(); i++) {
        // Ensure we do not have any bad KLT tracks (i.e., points are negative)
        if(pts_left_new[i].pt.x < 0 || pts_left_new[i].pt.y < 0 || pts_right_new[i].pt.x < 0 || pts_right_new[i].pt.y < 0)
            continue;
        // If it is a good track, and also tracked from left to right
        if(mask_ll[i] && mask_rr[i] && mask_lr[i]) {
            good_left.push_back(pts_left_new[i]);
            good_right.push_back(pts_right_new[i]);
            good_ids_left.push_back(ids_last[cam_id_left][i]);
            good_ids_right.push_back(ids_last[cam_id_right][i]);
        }
        // We can also track mono features, so lets handle them here
        //if(mask_ll[i]) {
        //    good_left.push_back(pts_left_new[i]);
        //    good_ids_left.push_back(ids_left_last[i]);
        //}
    }


    //===================================================================================
    //===================================================================================

    // Update our feature database, with theses new observations
    for(size_t i=0; i<good_left.size(); i++) {
        // Assert that our IDs are the same (i.e., stereo )
        assert(good_ids_left.at(i)==good_ids_right.at(i));
        // Try to undistort the point
        cv::Point2f npt_l = undistort_point(good_left.at(i).pt, cam_id_left);
        cv::Point2f npt_r = undistort_point(good_right.at(i).pt, cam_id_right);
        // Append to the database
        database->update_feature(good_ids_left.at(i), timestamp, cam_id_left,
                                 good_left.at(i).pt.x, good_left.at(i).pt.y,
                                 npt_l.x, npt_l.y);
        database->update_feature(good_ids_left.at(i), timestamp, cam_id_right,
                                 good_right.at(i).pt.x, good_right.at(i).pt.y,
                                 npt_r.x, npt_r.y);
    }

    // Move forward in time
    img_last[cam_id_left] = img_left.clone();
    img_last[cam_id_right] = img_right.clone();
    img_pyramid_last[cam_id_left] = imgpyr_left;
    img_pyramid_last[cam_id_right] = imgpyr_right;
    pts_last[cam_id_left] = good_left;
    pts_last[cam_id_right] = good_right;
    ids_last[cam_id_left] = good_ids_left;
    ids_last[cam_id_right] = good_ids_right;
    rT6 =  boost::posix_time::microsec_clock::local_time();


    // Timing information
    //ROS_INFO("[TIME-KLT]: %.4f seconds for pyramid",(rT2-rT1).total_microseconds() * 1e-6);
    //ROS_INFO("[TIME-KLT]: %.4f seconds for detection",(rT3-rT2).total_microseconds() * 1e-6);
    //ROS_INFO("[TIME-KLT]: %.4f seconds for temporal klt",(rT4-rT3).total_microseconds() * 1e-6);
    //ROS_INFO("[TIME-KLT]: %.4f seconds for stereo klt",(rT5-rT4).total_microseconds() * 1e-6);
    //ROS_INFO("[TIME-KLT]: %.4f seconds for feature DB update (%d features)",(rT6-rT5).total_microseconds() * 1e-6, (int)good_left.size());
    //ROS_INFO("[TIME-KLT]: %.4f seconds for total",(rT6-rT1).total_microseconds() * 1e-6);


}

// 传入当前帧的金字塔，当前帧已跟踪到的特征点以及特征点对应的id
void TrackKLT::perform_detection_monocular(const std::vector<cv::Mat> &img0pyr, std::vector<cv::KeyPoint> &pts0, std::vector<size_t> &ids0) {

    // Create a 2D occupancy grid for this current image
    // Note that we scale this down, so that each grid point is equal to a set of pixels
    // This means that we will reject points that less then grid_px_size points away then existing features
    Eigen::MatrixXi grid_2d_current = Eigen::MatrixXi::Zero((int)(img0pyr.at(0).cols / min_px_dist) + 10, (int)(img0pyr.at(0).rows / min_px_dist) + 10);
    auto it0 = pts0.begin();
    auto it2 = ids0.begin();
    while(it0 != pts0.end()) {
        // Get current left keypoint
        cv::KeyPoint kpt = *it0;
        // Check if this keypoint is near another point
        if(grid_2d_current((int)(kpt.pt.x/min_px_dist),(int)(kpt.pt.y/min_px_dist)) == 1) {   // 检测每个最小的grid中是否有特征点，保证相邻的两个特征点之间的距离大于min_px_dist
            it0 = pts0.erase(it0);                                                            // pts和ids已经剔除相距太近的冗余特征点
            it2 = ids0.erase(it2);
            continue;
        }
        // Else we are good, move forward to the next point
        grid_2d_current((int)(kpt.pt.x/min_px_dist),(int)(kpt.pt.y/min_px_dist)) = 1;
        it0++;
        it2++;
    }

    // First compute how many more features we need to extract from this image
    int num_featsneeded = num_features - (int)pts0.size();

    // If we don't need any features, just return
    if(num_featsneeded < 1)
        return;

    // Extract our features (use fast with griding)
    std::vector<cv::KeyPoint> pts0_ext;
    Grider_FAST::perform_griding(img0pyr.at(0), pts0_ext, num_featsneeded, grid_x, grid_y, threshold, true);              // 在每一个grid中检测fast角点

    // Create a 2D occupancy grid for this current image
    // Note that we scale this down, so that each grid point is equal to a set of pixels
    // This means that we will reject points that less then grid_px_size points away then existing features
    Eigen::MatrixXi grid_2d = Eigen::MatrixXi::Zero((int)(img0pyr.at(0).cols / min_px_dist) + 10, (int)(img0pyr.at(0).rows / min_px_dist) + 10);
    for(auto& kpt : pts0) {
        grid_2d((int)(kpt.pt.x/min_px_dist),(int)(kpt.pt.y/min_px_dist)) = 1;                                             // 对原来的特征点设置occupancy_grid
    }

    // Now, reject features that are close a current feature
    std::vector<cv::KeyPoint> kpts0_new;
    std::vector<cv::Point2f> pts0_new;
    for(auto& kpt : pts0_ext) {
        // See if there is a point at this location
        if(grid_2d((int)(kpt.pt.x/min_px_dist),(int)(kpt.pt.y/min_px_dist)) == 1)     // 将新检测到的特征点和原来的特征点融合到同一个occupancy_grid 中
            continue;
        // Else lets add it!
        kpts0_new.push_back(kpt);                                                     // 如果新加入的特征点和已存在的特征点以及当前的特征点都没有冲突的话，直接加入
        pts0_new.push_back(kpt.pt);
        grid_2d((int)(kpt.pt.x/min_px_dist),(int)(kpt.pt.y/min_px_dist)) = 1;
    }

    // Loop through and record only ones that are valid
    for(size_t i=0; i<pts0_new.size(); i++) {
        // update the uv coordinates
        kpts0_new.at(i).pt = pts0_new.at(i);
        // append the new uv coordinate
        pts0.push_back(kpts0_new.at(i));
        // move id foward and append this new point
        size_t temp = ++currid;                                                      // 给新检测到的特征点取一个id,并融合到原来的特征点中
        ids0.push_back(temp);
    }

}


void TrackKLT::perform_detection_stereo(const std::vector<cv::Mat> &img0pyr, const std::vector<cv::Mat> &img1pyr,
                                        std::vector<cv::KeyPoint> &pts0, std::vector<cv::KeyPoint> &pts1,
                                        std::vector<size_t> &ids0, std::vector<size_t> &ids1) {

    // Create a 2D occupancy grid for this current image
    // Note that we scale this down, so that each grid point is equal to a set of pixels
    // This means that we will reject points that less then grid_px_size points away then existing features
    Eigen::MatrixXi grid_2d_current = Eigen::MatrixXi::Zero((int)(img0pyr.at(0).cols / min_px_dist) + 10, (int)(img0pyr.at(0).rows / min_px_dist) + 10);
    auto it0 = pts0.begin();
    auto it1 = pts1.begin();
    auto it2 = ids0.begin();
    auto it3 = ids1.begin();
    while(it0 != pts0.end()) {
        // Get current left keypoint
        cv::KeyPoint kpt = *it0;
        // Check if this keypoint is near another point
        if(grid_2d_current((int)(kpt.pt.x/min_px_dist),(int)(kpt.pt.y/min_px_dist)) == 1) {
            it0 = pts0.erase(it0);
            it1 = pts1.erase(it1);
            it2 = ids0.erase(it2);
            it3 = ids1.erase(it3);
            continue;
        }
        // Else we are good, move forward to the next point
        grid_2d_current((int)(kpt.pt.x/min_px_dist),(int)(kpt.pt.y/min_px_dist)) = 1;
        it0++;
        it1++;
        it2++;
        it3++;
    }

    // First compute how many more features we need to extract from this image
    int num_featsneeded = num_features - (int)pts0.size();

    // If we don't need any features, just return
    if(num_featsneeded < 1)
        return;

    // Extract our features (use fast with griding)
    std::vector<cv::KeyPoint> pts0_ext;
    Grider_FAST::perform_griding(img0pyr.at(0), pts0_ext, num_featsneeded, grid_x, grid_y, threshold, true);


    // Create a 2D occupancy grid for this current image
    // Note that we scale this down, so that each grid point is equal to a set of pixels
    // This means that we will reject points that less then grid_px_size points away then existing features
    Eigen::MatrixXi grid_2d = Eigen::MatrixXi::Zero((int)(img0pyr.at(0).cols / min_px_dist) + 10, (int)(img0pyr.at(0).rows / min_px_dist) + 10);
    for(auto& kpt : pts0) {
        grid_2d((int)(kpt.pt.x/min_px_dist),(int)(kpt.pt.y/min_px_dist)) = 1;
    }

    // Now, reject features that are close a current feature
    std::vector<cv::KeyPoint> kpts0_new;
    std::vector<cv::Point2f> pts0_new;
    for(auto& kpt : pts0_ext) {
        // See if there is a point at this location
        if(grid_2d((int)(kpt.pt.x/min_px_dist),(int)(kpt.pt.y/min_px_dist)) == 1)
            continue;
        // Else lets add it!
        kpts0_new.push_back(kpt);
        pts0_new.push_back(kpt.pt);
        grid_2d((int)(kpt.pt.x/min_px_dist),(int)(kpt.pt.y/min_px_dist)) = 1;
    }

    // TODO: Project points from the left frame into the right frame
    // TODO: this will not work for large baseline systems.....
    std::vector<cv::KeyPoint> kpts1_new;
    std::vector<cv::Point2f> pts1_new;
    kpts1_new = kpts0_new;
    pts1_new = pts0_new;

    // If we don't have any new points, just return
    if(pts0_new.empty())
        return;

    // Now do KLT tracking to get the valid projections
    // Note: we have a pretty big window size here since our projection might be bad
    // Note: but this might cause failure in cases of repeated textures (eg. checkerboard)
    std::vector<uchar> mask;
    std::vector<float> error;
    cv::TermCriteria term_crit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 15, 0.01);
    cv::calcOpticalFlowPyrLK(img0pyr, img1pyr, pts0_new, pts1_new, mask, error, win_size, pyr_levels, term_crit, cv::OPTFLOW_USE_INITIAL_FLOW);

    // Loop through and record only ones that are valid
    for(size_t i=0; i<pts0_new.size(); i++) {
        if(mask[i] == 1) {
            // update the uv coordinates
            kpts0_new.at(i).pt = pts0_new.at(i);
            kpts1_new.at(i).pt = pts1_new.at(i);
            // append the new uv coordinate
            pts0.push_back(kpts0_new.at(i));
            pts1.push_back(kpts1_new.at(i));
            // move id foward and append this new point
            size_t temp = ++currid;
            ids0.push_back(temp);
            ids1.push_back(temp);
        }
    }

}


void TrackKLT::perform_matching(const std::vector<cv::Mat>& img0pyr, const std::vector<cv::Mat>& img1pyr,
                                std::vector<cv::KeyPoint>& kpts0, std::vector<cv::KeyPoint>& kpts1,
                                size_t id0, size_t id1,
                                std::vector<uchar>& mask_out) {

    // We must have equal vectors
    assert(kpts0.size() == kpts1.size());

    // Return if we don't have any points
    if(kpts0.empty() || kpts1.empty())
        return;

    // Convert keypoints into points (stupid opencv stuff)
    std::vector<cv::Point2f> pts0, pts1;
    for(size_t i=0; i<kpts0.size(); i++) {
        pts0.push_back(kpts0.at(i).pt);
        pts1.push_back(kpts1.at(i).pt);
    }

    // If we don't have enough points for ransac just return empty
    // We set the mask to be all zeros since all points failed RANSAC
    if(pts0.size() < 10) {
        for(size_t i=0; i<pts0.size(); i++)
            mask_out.push_back((uchar)0);
        return;
    }

    // Now do KLT tracking to get the valid new points
    std::vector<uchar> mask_klt;
    std::vector<float> error;
    cv::TermCriteria term_crit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 15, 0.01);
    cv::calcOpticalFlowPyrLK(img0pyr, img1pyr, pts0, pts1, mask_klt, error, win_size, pyr_levels, term_crit, cv::OPTFLOW_USE_INITIAL_FLOW);
    // 对原始图像上的特征点进行光流跟踪，pts1现在变成光流跟踪后的坐标

    // Normalize these points, so we can then do ransac
    // We don't want to do ransac on distorted image uvs since the mapping is nonlinear
    std::vector<cv::Point2f> pts0_n, pts1_n;
    for(size_t i=0; i<pts0.size(); i++) {
        pts0_n.push_back(undistort_point(pts0.at(i),id0));                    // 对前一帧特征点去畸变,得到去畸变归一化平面坐标
        pts1_n.push_back(undistort_point(pts1.at(i),id1));                    // 对当前帧特征点去畸变,得到去畸变归一化平面坐标
    }

    // Do RANSAC outlier rejection (note since we normalized the max pixel error is now in the normalized cords)
    std::vector<uchar> mask_rsc;
    double max_focallength_img0 = std::max(camera_k_OPENCV.at(id0)(0,0),camera_k_OPENCV.at(id0)(1,1));
    double max_focallength_img1 = std::max(camera_k_OPENCV.at(id1)(0,0),camera_k_OPENCV.at(id1)(1,1));
    double max_focallength = std::max(max_focallength_img0,max_focallength_img1);
    cv::findFundamentalMat(pts0_n, pts1_n, cv::FM_RANSAC, 1/max_focallength, 0.999, mask_rsc);          // 通过OpenCV FindFundamentalMat Ransac来剔除噪声点，输入是去畸变后的归一化平面坐标

    // Loop through and record only ones that are valid
    for(size_t i=0; i<mask_klt.size(); i++) {
        auto mask = (uchar)((i < mask_klt.size() && mask_klt[i] && i < mask_rsc.size() && mask_rsc[i])? 1 : 0);  // 如果光流跟踪和ransac都是内点才认为该特征点成功匹配
        mask_out.push_back(mask);
    }

    // Copy back the updated positions
    for(size_t i=0; i<pts0.size(); i++) {                                    // 返回成功匹配的特征点
        kpts0.at(i).pt = pts0.at(i);
        kpts1.at(i).pt = pts1.at(i);
    }

}



