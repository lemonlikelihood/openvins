//
// Created by SENSETIME\yuanjin on 2020/1/14.
//

#include "LoopCloser.h"
#include <iostream>
#include <opencv2/core/core.hpp>

using namespace ov_msckf;

void LoopCloser::load_vocabulary(std::string voc_path) {
    voc = new BriefVocabulary(voc_path);
    db.setVocabulary(*voc, false, 0);
    std::cout << "------------------load_vocabulary successful------------------" << std::endl;

}

void LoopCloser::feed_monocular(double timestamp, cv::Mat img, size_t cam_id, TrackBase *trackBase) {
    cur_keyframe = new KeyFrame(timestamp, img, cam_id, trackBase, frame_id);
    cur_keyframe->computeBRIEFPoint();
    ransac_loop();
    if (frame_id % step == 0) {
        add_keyframe_into_voc(cur_keyframe);
    }
    frame_id++;
}

int LoopCloser::detect_loop(KeyFrame *keyframe, int frame_index) {
    // put image into image_pool; for visualization
    cv::Mat compressed_image;
    QueryResults ret;

    db.query(keyframe->brief_descriptors, ret, 4, frame_index - 50);
    bool find_loop = false;
    std::cout<<"frame_index: "<<frame_index<<"ret_size: "<<ret.size()<<std::endl;
    for(int i=0;i<ret.size();i++){
        std::cout<<"ret "<<i <<" frame_id: "<< get_keyframe(ret[i].Id)->frame_id << " Score: "<<ret[i].Score<<std::endl;
    }
    if (ret.size() > 1 && ret[0].Score > 0.05)
        for (unsigned int i = 1; i < ret.size(); i++) {
            //if (ret[i].Score > ret[0].Score * 0.3)
            if (ret[i].Score > 0.015) {
                find_loop = true;
                break;
            }
        }

    if (find_loop && frame_index > 50) {
        int min_index = -1;
        for (unsigned int i = 0; i < ret.size(); i++) {
            if (min_index == -1 || (ret[i].Id < min_index && ret[i].Score > 0.015))
                min_index = ret[i].Id;
        }
        return min_index;
    } else
        return -1;
}

KeyFrame *LoopCloser::get_keyframe(int index) {
//    unique_lock<mutex> lock(m_keyframelist);
    list<KeyFrame *>::iterator it = keyFrameList.begin();
    for (; it != keyFrameList.end(); it++) {
        if ((*it)->keyframe_id == index)
            break;
    }
    if (it != keyFrameList.end())
        return *it;
    else
        return nullptr;
}


void LoopCloser::add_keyframe_into_voc(KeyFrame *keyframe) {
//    // put image into image_pool; for visualization
//    cv::Mat compressed_image;
//    if (DEBUG_IMAGE)
//    {
//        int feature_num = keyframe->keypoints.size();
//        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
//        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
//        image_pool[keyframe->index] = compressed_image;
//    }

    db.add(keyframe->brief_descriptors);
    keyframe->set_keyframe_id(keyframe_id++);
    keyFrameList.push_back(keyframe);
}

bool LoopCloser::ransac_loop() {
    int old_keyframe_id = detect_loop(cur_keyframe, frame_id);
    if(old_keyframe_id == -1){
        std::cout<<"did not find loopcloser at frame:"<<cur_keyframe->frame_id<<std::endl;
        return false;
    }

    KeyFrame *old_keyframe = get_keyframe(old_keyframe_id);

    std::cout<<"old_frame: "<<old_keyframe->frame_id<<std::endl;

    std::vector<cv::Point2f> matched_2d_old;
    std::vector<cv::Point2f> matched_2d_cur;

    bool is_find_loop = false;
    is_find_loop =  cur_keyframe->findConnection(old_keyframe,matched_2d_old,matched_2d_cur);
    if (is_find_loop) {
        int feature_num = cur_keyframe->keypoints.size();
        cv::Mat compressed_image = cur_keyframe->image.clone();
        putText(compressed_image, "image:" + to_string(cur_keyframe->frame_id) + " feature_num:"
                                  + to_string(feature_num) + "keyframe_size:" + to_string(keyframe_id),
                cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255));
        cv::Mat loop_result;
        loop_result = compressed_image.clone();
//        putText(loop_result, "neighbour score:" + to_string(ret[0].Score), cv::Point2f(10, 50),
//                    CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));

        cv::Mat tmp_image = (old_keyframe->image).clone();
        putText(tmp_image, "best index:  " + to_string(old_keyframe->frame_id),
                cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255));
        cv::hconcat(loop_result, tmp_image, loop_result);
        for(int i=0;i<matched_2d_cur.size();i++){
            cv::RNG rng = cv::RNG(cv::getTickCount());
            int r = rng.uniform(0,255);
            int g = rng.uniform(0,255);
            int b = rng.uniform(0,255);
            cv::line(loop_result,matched_2d_cur[i],cv::Point2f(matched_2d_old[i].x+compressed_image.cols,matched_2d_old[i].y),cv::Scalar(r,g,b), 2);
        }
        if (DEBUG_IMAGE) {
            cv::imshow("loop_result", loop_result);
            cv::waitKey(20);
        }

    }
    return is_find_loop;
}