#include "leastSquares.h"

// TODO: calcola la differenza tramite prodotto di matrici omogenee
Vector3d LS::compute_error(Tricycle tricycle, Vector3d tracker_pose)
{
    return utils::t2v( utils::v2t(tricycle.get_sensor_pose()).inverse() * utils::v2t(tracker_pose) );
}

MatrixXd LS::compute_numeric_jacobian(Tricycle tricycle, uint32_t actual_tick_traction, uint32_t next_tick_traction, uint32_t actual_tick_steer)
{
    double epsilon = 1e-9;
    double inv_eps2= 0.5/epsilon;
    std::vector<double> parameters = tricycle.get_parameters_to_calibrate();
    MatrixXd J(3,parameters.size());
    // Kinematic parameters of the robot
    for(int i = 0; i < 4; ++i){
        std::vector<double> parameters_plus = parameters;
        std::vector<double> parameters_minus = parameters;

        parameters_plus[i] += epsilon;
        parameters_minus[i] -= epsilon;


        // TODO: calcola la differenza tramite prodotto di matrici omogenee
        Vector3d sensor_pose_plus = std::get<2>(tricycle.predict(parameters_plus,
                                                                 actual_tick_traction,
                                                                 next_tick_traction, 
                                                                 actual_tick_steer));

        Vector3d sensor_pose_minus = std::get<2>(tricycle.predict(parameters_minus,
                                                                 actual_tick_traction,
                                                                 next_tick_traction, 
                                                                 actual_tick_steer));

        // Differenza matriciale
        // Vector3d column = inv_eps2 * utils::t2v(utils::v2t(sensor_pose_plus).inverse() * utils::v2t(sensor_pose_minus));
        // J.col(i) = column;

        // Differenza numerica
        J.col(i) = inv_eps2*(sensor_pose_plus - sensor_pose_minus);
    }
    // Parameters (pose) of the sensor w.r.t. robot
    for(int i=0; i<3; ++i){
        Vector3d noise = Vector3d::Zero();
        noise[i] = epsilon;
        Vector3d sensor_pose = std::get<2>(tricycle.predict(parameters,
                                                            actual_tick_traction,
                                                            next_tick_traction, 
                                                            actual_tick_steer));
        
        Vector3d sensor_pose_plus = utils::t2v(utils::v2t(sensor_pose) * utils::v2t(noise));
        Vector3d sensor_pose_minus = utils::t2v(utils::v2t(sensor_pose) * utils::v2t(-noise));

        // Differenza matriciale
        // Vector3d column = inv_eps2 * utils::t2v(utils::v2t(sensor_pose_plus).inverse() * utils::v2t(sensor_pose_minus));
        // J.col(i+4) = column;

        // Differenza numerica
        J.col(i+4) = inv_eps2*(sensor_pose_plus - sensor_pose_minus);
    }
    return J;
}
