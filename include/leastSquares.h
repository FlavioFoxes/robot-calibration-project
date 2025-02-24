#pragma once

#include <Eigen/Geometry>
#include "utils.h"
#include "tricycle.h"

namespace LS{
    using namespace std;
    using namespace Eigen;
    using namespace utils;

    Vector3d compute_error(Tricycle tricycle, Affine2d observation, Vector3d d_pose);
    MatrixXd compute_numeric_jacobian(Tricycle tricycle, 
                                      Vector3d tricycle_pose,
                                      uint32_t actual_tick_traction, 
                                      uint32_t next_tick_traction, 
                                      uint32_t actual_tick_steer, 
                                      Affine2d observation,
                                      Vector3d d_pose);
    std::vector<double> calibrate_parameters(VectorXd dx, std::vector<double> parameters);
}