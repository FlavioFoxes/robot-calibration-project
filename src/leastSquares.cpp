#include "leastSquares.h"

Vector3d LS::compute_error(Tricycle tricycle, Affine2d observation, Vector3d d_pose)
{
    // Compute displacement of the sensor (prediction)
    Affine2d d_pose_sensor = v2t(tricycle.get_sensor_pose_rel()).inverse() * v2t(d_pose) * v2t(tricycle.get_sensor_pose_rel());
    // Error is not simply a normal difference, because 
    // it is a manifold (non-Euclidean space)
    return t2v(observation.inverse() * d_pose_sensor);
}

MatrixXd LS::compute_numeric_jacobian(Tricycle tricycle, 
                                      Vector3d tricycle_pose, 
                                      uint32_t actual_tick_traction,
                                      uint32_t next_tick_traction,
                                      uint32_t actual_tick_steer, 
                                      Vector3d d_pose)
{

    double epsilon = 1e-11;
    std::vector<double> parameters = tricycle.get_parameters_to_calibrate();

    // Instantiate Jacobian matrix
    MatrixXd J(3,7);

    // For each robot kinematic parameter 
    // (k_steer, k_traction, steer_offset, baseline)
    for(int i=0; i<4; ++i){
        // Copy robot parameters
        std::vector<double> parameters_plus = parameters;
        std::vector<double> parameters_minus = parameters;
        // Add/remove little epsilon to the interested parameter
        parameters_plus[i] += epsilon;
        parameters_minus[i] -= epsilon;

        // Compute robot displacement using altered (plus) parameters
        Vector3d d_pose_plus = tricycle.predict(parameters_plus,
                                                tricycle_pose,
                                                actual_tick_traction,
                                                next_tick_traction,
                                                actual_tick_steer);

        // Compute robot displacement using altered (minus) parameters
        Vector3d d_pose_minus = tricycle.predict(parameters_minus,
                                                 tricycle_pose,
                                                 actual_tick_traction,
                                                 next_tick_traction,
                                                 actual_tick_steer);

        // Compute corresponding sensor displacement (prediction)
        Affine2d d_pose_sensor_plus = v2t(tricycle.get_sensor_pose_rel()).inverse() * v2t(d_pose_plus) * v2t(tricycle.get_sensor_pose_rel());
        Affine2d d_pose_sensor_minus = v2t(tricycle.get_sensor_pose_rel()).inverse() * v2t(d_pose_minus) * v2t(tricycle.get_sensor_pose_rel());

        // Compute numeric derivative w.r.t. i-th parameter
        // (i-th column of jacobian matrix)
        Vector3d column = t2v(d_pose_sensor_plus) - t2v(d_pose_sensor_minus);
        column /= (2*epsilon);

        // Insert column in the jacobian
        J.col(i) = column;                                                            
    }
    // For each sensor parameter w.r.t. robot kinematic center
    // (x, y, theta)
    for(int i=0; i<3; ++i){
        Vector3d noise = Vector3d::Zero();
        noise[i] += epsilon;

        // Add/remove little epsilon to the interested parameter
        // NOTE: it is a manifold, so normal addition is not possible
        Affine2d sensor_pose_rel_plus = v2t(noise) * v2t(tricycle.get_sensor_pose_rel());
        Affine2d sensor_pose_rel_minus = v2t(-noise) * v2t(tricycle.get_sensor_pose_rel());
        
        // Compute sensor displacement using altered (plus/minus) parameters
        // NOTE: it's not necessary to compute d_pose, because it is equals to 
        //       the one applied to the robot (passed as argument in input)
        Affine2d d_pose_sensor_plus = sensor_pose_rel_plus.inverse() * v2t(d_pose) * sensor_pose_rel_plus;
        Affine2d d_pose_sensor_minus = sensor_pose_rel_minus.inverse() * v2t(d_pose) * sensor_pose_rel_minus;

        // Compute numeric derivative w.r.t. i-th parameter
        // (i-th column of jacobian matrix)
        Vector3d column = t2v(d_pose_sensor_plus) - t2v(d_pose_sensor_minus);
        column /= (2*epsilon);

        // Insert column in the jacobian
        J.col(i+4) = column;
    }
    return J;
}

std::vector<double> LS::calibrate_parameters(VectorXd dx, std::vector<double> parameters)
{
    // Take nominal values of the parameters to calibrate
    std::vector<double> calibrated_parameters = parameters;

    // Calibrate kinematic parameters of the robot
    for(int i=0; i<4; ++i){
        calibrated_parameters[i] = parameters[i] + dx[i];
    }
    
    // Sensor pose relative to the kinematic center of the robot
    Vector3d sensor_pose_rel = Vector3d(parameters[4], parameters[5], parameters[6]);
    // Unpack calibrations (adjustments) about relative sensor pose w.r.t. robot kinematic center
    Vector3d d_sensor = Vector3d(dx[4], dx[5], dx[6]);

    // Calibrate relative sensor pose w.r.t. robot kinematic center
    // and save it in the calibrated_parameters vector
    Vector3d sensor_pose_calibrated = t2v( v2t(d_sensor) * v2t(sensor_pose_rel) );
    calibrated_parameters[4] = sensor_pose_calibrated[0];
    calibrated_parameters[5] = sensor_pose_calibrated[1];
    calibrated_parameters[6] = sensor_pose_calibrated[2];

    return calibrated_parameters;
}

/*
    TODO: compute_analytic_jacobian

    The function must compute the analytic jacobian matrix of the prediction 
    function w.r.t. parameters to calibrate.
    Check if it is necessary to use explicit or matricial form to compute
    the derivatives

    MatrixXd LS::compute_analytic_jacobian(Tricycle tricycle, 
                                           uint32_t actual_tick_traction, 
                                           uint32_t next_tick_traction, 
                                           uint32_t actual_tick_steer)

*/