#include "leastSquares.h"

Vector3d LS::compute_error(Tricycle tricycle, Vector3d tracker_pose)
{
    return tricycle.get_sensor_pose() - tracker_pose;
}

void LS::compute_numeric_jacobian(Tricycle tricycle)
{
    double epsilon = 1e-3;
    double inv_eps2= 0.5/epsilon;

}
