#include "leastSquares.h"

Vector3d LS::compute_error(Tricycle tricycle, Vector3d tracker_pose)
{
    return tricycle.get_sensor_pose() - tracker_pose;
}

void LS::compute_numeric_jacobian(Tricycle tricycle)
{
    double epsilon = 1e-3;
    double inv_eps2= 0.5/epsilon;

    for(int i = 0; i < parameters.size(); ++i){
        std::vector<double> parameters_plus = tricycle.get_parameters_to_calibrate();
        std::vector<double> parameters_minus = tricycle.get_parameters_to_calibrate();

        parameters_plus[i] += epsilon;
        parameters_minus[i] -= epsilon;

        // TODO: compute numeric jacobian ( inv_eps2 * (h(parameters_plus) - h(parameters_minus)) )
        

    }
}
