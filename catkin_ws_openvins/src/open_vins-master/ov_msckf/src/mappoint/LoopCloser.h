//
// Created by SENSETIME\yuanjin on 2020/1/14.
//

#ifndef CATKIN_WS_OPENVINS_LOOPCLOSER_H
#define CATKIN_WS_OPENVINS_LOOPCLOSER_H

#include <list>
#include <vector>
#include "ThirdParty/DBoW/DBoW2.h"
#include "ThirdParty/DVision/DVision.h"
#include "ThirdParty/DBoW/TemplatedDatabase.h"
#include "ThirdParty/DBoW/TemplatedVocabulary.h"

#include "keyframe/KeyFrame.h"
#include "track/TrackBase.h"

#define DEBUG_IMAGE true
using namespace DVision;
using namespace DBoW2;
using namespace ov_core;

namespace ov_msckf {
    class LoopCloser{
    public:
        LoopCloser()= default;
        ~LoopCloser()= default;
        void load_vocabulary(std::string voc_path);
        void feed_monocular(double timestamp, cv::Mat img, size_t cam_id,TrackBase * trackBase);
        int detect_loop(KeyFrame* keyframe, int frame_index);
        void add_keyframe_into_voc(KeyFrame* keyframe);
        bool ransac_loop();
        KeyFrame* get_keyframe(int index);

    private:
        BriefDatabase db;
        BriefVocabulary* voc;

        std::list<KeyFrame*> keyFrameList;
        int frame_id = 0;
        int step = 20;
        int keyframe_id = 0;
        KeyFrame *cur_keyframe = nullptr;
    };
}
#endif //CATKIN_WS_OPENVINS_LOOPCLOSER_H
