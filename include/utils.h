#pragma once

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include <assert.h>

// Definition of ASSERT with custom message
#define ASSERT(condition, message) \
   do { \
      assert(condition && message); \
   } while (0)


namespace utils{
    using namespace std;

    struct Dataset{
        vector<double> time;
        vector<uint32_t> ticks_steer;
        vector<uint32_t> ticks_traction;
        vector<float> model_pose_x;
        vector<float> model_pose_y;
        vector<float> model_pose_theta;
        vector<float> tracker_pose_x;
        vector<float> tracker_pose_y;
        vector<float> tracker_pose_theta;       
    };

    Dataset create_dataset(string name_file);
}