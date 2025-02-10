#pragma once

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include <assert.h>
#include <Eigen/Geometry>



// Definition of ASSERT with custom message
#define ASSERT(condition, message) \
   do { \
      assert(condition && message); \
   } while (0)


namespace utils{
    using namespace std;
    using namespace Eigen;


    struct Dataset{
        vector<double> time;
        vector<uint32_t> ticks_steer;
        vector<uint32_t> ticks_traction;
        vector<double> model_pose_x;
        vector<double> model_pose_y;
        vector<double> model_pose_theta;
        vector<double> tracker_pose_x;
        vector<double> tracker_pose_y;
        vector<double> tracker_pose_theta;       
    };

    Dataset create_dataset(string name_file);
    void write_pose(std::string name_file, Vector3d pose);

    
    Affine2d v2t(Vector3d pose);
    
    Vector3d t2v(Affine2d t);

}