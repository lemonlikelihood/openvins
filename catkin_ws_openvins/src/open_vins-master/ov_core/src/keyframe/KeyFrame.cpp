//
// Created by SENSETIME\yuanjin on 2020/1/14.
//

#include "KeyFrame.h"

using namespace ov_core;

template <typename Derived>
static void reduceVector(vector<Derived> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void
BriefExtractor::operator()(const cv::Mat &im, vector <cv::KeyPoint> &keys, vector <BRIEF::bitset> &descriptors) const {
    m_brief.compute(im, keys, descriptors);
}

BriefExtractor::BriefExtractor(const std::string &pattern_file) {
    // The DVision::BRIEF extractor computes a random pattern by default when
    // the object is created.
    // We load the pattern that we used to build the vocabulary, to make
    // the descriptors compatible with the predefined vocabulary

    // loads the pattern
    cv::FileStorage fs(pattern_file.c_str(), cv::FileStorage::READ);
    if (!fs.isOpened()) throw string("Could not open file ") + pattern_file;

    vector<int> x1, y1, x2, y2;
    fs["x1"] >> x1;
    fs["x2"] >> x2;
    fs["y1"] >> y1;
    fs["y2"] >> y2;

    m_brief.importPairs(x1, y1, x2, y2);
}

KeyFrame::KeyFrame(double _timestamp, cv::Mat _img, size_t _cam_id, TrackBase *_trackBase, std::size_t _frame_id) :
        timestamp(_timestamp), image(_img), cam_id(_cam_id), trackFEATS(_trackBase), frame_id(_frame_id) {
    keyframe_id = -1;
}

void KeyFrame::set_keyframe_id(std::size_t id) {
    keyframe_id = id;
}

std::size_t KeyFrame::get_keyframe_id() {
    return keyframe_id;
}

//void KeyFrame::computeWindowBRIEFPoint() {
//    string BRIEF_PATTERN_FILE = "/home/SENSETIME/yuanjin/Workspace/catkin_ws_openvins/src/open_vins-master/ov_core/src/ThirdParty/brief_pattern.yml";
//    BriefExtractor extractor(BRIEF_PATTERN_FILE);
//    for (int i = 0; i < (int) point_2d_uv.size(); i++) {
//        cv::KeyPoint key;
//        key.pt = point_2d_uv[i];
//        window_keypoints.push_back(key);
//    }
//    extractor(image, window_keypoints, window_brief_descriptors);
//}

void KeyFrame::computeBRIEFPoint() {
    string BRIEF_PATTERN_FILE = "/home/SENSETIME/yuanjin/Workspace/catkin_ws_openvins/src/open_vins-master/ov_core/src/ThirdParty/brief_pattern.yml";
    BriefExtractor extractor(BRIEF_PATTERN_FILE);
    const int fast_th = 20; // corner detector response threshold
    cv::FAST(image, keypoints, fast_th, true);

    extractor(image, keypoints, brief_descriptors);
    for (int i = 0; i < (int) keypoints.size(); i++) {
        Eigen::Vector3d tmp_p;
        point_2d_uv.push_back(keypoints[i].pt);
        point_2d_norm.push_back(trackFEATS->undistort_point(keypoints[i].pt, cam_id));
    }
}


int KeyFrame::HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b) {
    BRIEF::bitset xor_of_bitset = a ^b;
    int dis = xor_of_bitset.count();
    return dis;
}

// 将window_descriptor和descriptors_old里面所有的描述符进行逐个匹配，查找最佳的匹配
bool KeyFrame::searchInAera(const BRIEF::bitset window_descriptor,
                            const std::vector<BRIEF::bitset> &descriptors_old,
                            const std::vector<cv::Point2f> &point_2d_uv_old,
                            const std::vector<cv::Point2f> &point_2d_uv_norm_old,
                            cv::Point2f &best_match,
                            cv::Point2f &best_match_norm) {
    cv::Point2f best_pt;
    int bestDist = 128;
    int bestIndex = -1;
    for (int i = 0; i < (int) descriptors_old.size(); i++) {

        int dis = HammingDis(window_descriptor, descriptors_old[i]);
        if (dis < bestDist) {
            bestDist = dis;
            bestIndex = i;
        }
    }
    //printf("best dist %d", bestDist);
    if (bestIndex != -1 && bestDist < 80) {
        best_match = point_2d_uv_old[bestIndex];
        best_match_norm = point_2d_uv_norm_old[bestIndex];
        return true;
    } else
        return false;
}

void KeyFrame::searchByBRIEFDes(std::vector<cv::Point2f> &matched_2d_old,
                                std::vector<cv::Point2f> &matched_2d_old_norm,
                                std::vector<uchar> &status,
                                const std::vector<BRIEF::bitset> &descriptors_old,
                                const std::vector<cv::Point2f> &point_2d_uv_old,
                                const std::vector<cv::Point2f> &point_2d_norm_old){
    for (int i = 0; i < (int) brief_descriptors.size(); i++) {
        cv::Point2f pt(0.f, 0.f);
        cv::Point2f pt_norm(0.f, 0.f);
        if (searchInAera(brief_descriptors[i], descriptors_old, point_2d_uv_old, point_2d_norm_old, pt, pt_norm))
            status.push_back(1);
        else
            status.push_back(0);
        matched_2d_old.push_back(pt);
        matched_2d_old_norm.push_back(pt_norm);
    }
}

bool KeyFrame::findConnection(ov_core::KeyFrame *old_kf,std::vector<cv::Point2f> &matched_2d_old,
                              std::vector<cv::Point2f> &matched_2d_cur) {
    //vector <cv::Point2f> matched_2d_cur, matched_2d_old;
    vector <cv::Point2f> matched_2d_cur_norm, matched_2d_old_norm;
    vector <cv::Point3f> matched_3d;
    vector<double> matched_id;
    vector <uchar> status;

//    matched_3d = point_3d;
    matched_2d_cur = point_2d_uv;
    matched_2d_cur_norm = point_2d_norm;
//    matched_id = point_id;

    searchByBRIEFDes(matched_2d_old, matched_2d_old_norm, status, old_kf->brief_descriptors, old_kf->point_2d_uv,
                     old_kf->point_2d_norm);
    reduceVector(matched_2d_cur, status);
    reduceVector(matched_2d_old, status);
    reduceVector(matched_2d_cur_norm, status);
    reduceVector(matched_2d_old_norm, status);
//    reduceVector(matched_3d, status);
//    reduceVector(matched_id, status);

    status.clear();

    if ((int)matched_2d_cur.size() > MIN_LOOP_NUM){
        status.clear();
        // Do RANSAC outlier rejection (note since we normalized the max pixel error is now in the normalized cords)
        std::map<size_t, cv::Matx33d> camera_k_OPENCV = trackFEATS->get_camera_k_OPENCV();
        double max_focallength_img0 = std::max(camera_k_OPENCV.at(old_kf->cam_id)(0,0),camera_k_OPENCV.at(old_kf->cam_id)(1,1));
        double max_focallength_img1 = std::max(camera_k_OPENCV.at(cam_id)(0,0),camera_k_OPENCV.at(cam_id)(1,1));
        double max_focallength = std::max(max_focallength_img0,max_focallength_img1);
        cv::findFundamentalMat(matched_2d_old_norm, matched_2d_cur_norm, cv::FM_RANSAC, 1/max_focallength, 0.999, status);          // 通过OpenCV FindFundamentalMat Ransac来剔除噪声点，输入是去畸变后的归一化平面坐标
        reduceVector(matched_2d_cur, status);
        reduceVector(matched_2d_old, status);
        reduceVector(matched_2d_cur_norm, status);
        reduceVector(matched_2d_old_norm, status);
        if((int)matched_2d_cur.size() > MIN_LOOP_NUM){
            return true;
        } else return false;
    } else return false;
}