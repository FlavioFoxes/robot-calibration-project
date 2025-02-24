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


/*
utils namespace.
It defines useful things for the project
*/
namespace utils{
    using namespace std;
    using namespace Eigen;

    /*
    Dataset definition.
    It contains std::vector for each information field in the dataset.
    In detail:
        - time:                     timestamps
        - ticks_steer:              ticks of the steering encoder
        - tick_traction:            ticks of the traction encoder
        - model_pose_x:             x-coordinate of the robot pose
        - model_pose_y:             y-coordinate of the robot pose
        - model_pose_theta:         theta-coordinate of the robot pose
        - tracker_pose_x:           x-coordinate of the tracker pose
        - tracker_pose_y:           y-coordinate of the tracker pose
        - tracker_pose_theta:       theta-coordinate of the tracker pose

    NOTE: At each index corresponds information belonging to the same timestamp
    */
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

    /*
    Create dataset, starting from dataset/dataset.txt file
    */
    Dataset create_dataset(string name_file);

    /*
    Write the trajectory in a file, for plotting through Python
    */
    void write_trajectory(std::string name_file, std::vector<Vector3d> trajectory);

    /*
    Vector-to-Transformation matrix function
    */
    Affine2d v2t(Vector3d pose);
    
    /*
    Transformation-to-Vector function
    */
    Vector3d t2v(Affine2d t);

}