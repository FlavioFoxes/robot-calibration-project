#pragma once

#include <Eigen/Geometry>
#include "utils.h"
#include "tricycle.h"

/*
Least Squares namespace.
It contains the functions to apply Gauss-Newton method in our problem.
*/
namespace LS{
    using namespace std;
    using namespace Eigen;
    using namespace utils;

    /*
    Compute the error between the prediction and the observation.
    NOTE: d_pose is not the prediction, but the displacement of the robot;
          prediction is the displacement of the sensor (computed directly inside)
    */
    Vector3d compute_error(Tricycle tricycle, Affine2d observation, Vector3d d_pose);

    /*
    Compute the numeric jacobian of the error w.r.t. parameters to calibrate
    */
    MatrixXd compute_numeric_jacobian(Tricycle tricycle, 
                                      Vector3d tricycle_pose,
                                      uint32_t actual_tick_traction, 
                                      uint32_t next_tick_traction, 
                                      uint32_t actual_tick_steer, 
                                      Vector3d d_pose);

    /*
    Calibrate parameters, applying each adjustment dx to the corresponding parameter
    */
    std::vector<double> calibrate_parameters(VectorXd dx, std::vector<double> parameters);
}