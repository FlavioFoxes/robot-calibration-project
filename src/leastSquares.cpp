#include "leastSquares.h"

Vector3d LS::compute_error(Tricycle tricycle, Vector3d tracker_pose)
{
    return tricycle.get_sensor_pose() - tracker_pose;
}

MatrixXd LS::compute_numeric_jacobian(Tricycle tricycle, uint32_t actual_tick_traction, uint32_t next_tick_traction, uint32_t actual_tick_steer)
{
    double epsilon = 1e-3;
    double inv_eps2= 0.5/epsilon;
    std::vector<double> parameters = tricycle.get_parameters_to_calibrate();
    MatrixXd J(3,parameters.size());
    for(int i = 0; i < parameters.size(); ++i){
        std::vector<double> parameters_plus = parameters;
        std::vector<double> parameters_minus = parameters;

        parameters_plus[i] += epsilon;
        parameters_minus[i] -= epsilon;



        Vector3d sensor_pose_plus = std::get<2>(tricycle.predict(parameters_plus,
                                                                 actual_tick_traction,
                                                                 next_tick_traction, 
                                                                 actual_tick_steer));

        
        Vector3d sensor_pose_minus = std::get<2>(tricycle.predict(parameters_minus,
                                                                 actual_tick_traction,
                                                                 next_tick_traction, 
                                                                 actual_tick_steer));
        

        //         std::tuple<double, Vector3d, Vector3d> predict(std::vector<double> parameters,
        //                                                uint32_t tick_traction,
        //                                                uint32_t next_tick_traction, 
        //                                                uint32_t tick_steer);

        // TODO: compute numeric jacobian ( inv_eps2 * (h(parameters_plus) - h(parameters_minus)) )
        Vector3d column = inv_eps2 * (sensor_pose_plus - sensor_pose_minus);
        J.col(i) = column;
    }
    return J;
}
