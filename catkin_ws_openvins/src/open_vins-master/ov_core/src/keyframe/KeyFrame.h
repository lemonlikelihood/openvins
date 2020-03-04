//
// Created by SENSETIME\yuanjin on 2020/1/14.
//

#ifndef CATKIN_WS_OPENVINS_KEYFRAME_H
#define CATKIN_WS_OPENVINS_KEYFRAME_H
#include <vector>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>

#include "../track/TrackBase.h"
#include "ThirdParty/DBoW/DBoW2.h"
#include "ThirdParty/DVision/DVision.h"
#define MIN_LOOP_NUM 25

using namespace Eigen;
using namespace std;
using namespace DVision;

namespace ov_core{

    class TrackBase;

    class BriefExtractor
    {
    public:
        virtual void operator()(const cv::Mat &im, vector<cv::KeyPoint> &keys, vector<BRIEF::bitset> &descriptors) const;
        BriefExtractor(const std::string &pattern_file);

        DVision::BRIEF m_brief;
    };

    class KeyFrame{
    public:

        KeyFrame(double _timestamp, cv::Mat _img, size_t _cam_id, TrackBase * _trackBase, std::size_t _frame_id);
        void setPoint(vector<cv::Point2f> &_point_2d_uv);
        void set_keyframe_id(std::size_t id);
        std::size_t get_keyframe_id();
        void computeBRIEFPoint();

//        void computeWindowBRIEFPoint();

        int HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b);
        bool findConnection(KeyFrame* old_kf,std::vector<cv::Point2f> &matched_2d_old,
                            std::vector<cv::Point2f> &matched_2d_cur);
        void searchByBRIEFDes(std::vector<cv::Point2f> &matched_2d_old,
                              std::vector<cv::Point2f> &matched_2d_old_norm,
                              std::vector<uchar> &status,
                              const std::vector<BRIEF::bitset> &descriptors_old,
                              const std::vector<cv::Point2f> &point_2d_uv_old,
                              const std::vector<cv::Point2f> &point_2d_norm_old);

        bool searchInAera(const BRIEF::bitset window_descriptor,
                          const std::vector<BRIEF::bitset> &descriptors_old,
                          const std::vector<cv::Point2f> &point_2d_uv_old,
                          const std::vector<cv::Point2f> &point_2d_uv_norm_old,
                          cv::Point2f &best_match,
                          cv::Point2f &best_match_norm);

//        int num_features;

        TrackBase* trackFEATS = nullptr;

        std::size_t cam_id;
        double timestamp;
        cv::Mat image;

        std::size_t keyframe_id;                                                 // 加入database后才有id,默认为-1
        std::size_t frame_id;

        vector<cv::Point3f> point_3d;
        vector<cv::Point2f> point_2d_uv;                                   // 图像平面原始坐标
        vector<cv::Point2f> point_2d_norm;                                 // 归一化平面坐标

//        vector<double> point_id;
        vector<cv::KeyPoint> keypoints;
//        vector<cv::KeyPoint> keypoints_norm;
        vector<BRIEF::bitset> brief_descriptors;
    };


}
#endif //CATKIN_WS_OPENVINS_KEYFRAME_H
