#pragma once

#include <Eigen/Geometry>
#include "utils.h"
#include "tricycle.h"

namespace LS{
    using namespace std;
    using namespace Eigen;

    Vector3d compute_error(Tricycle tricycle, Vector3d tracker_pose);
    void compute_numeric_jacobian(Tricycle tricycle);
}