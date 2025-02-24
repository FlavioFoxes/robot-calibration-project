#include "leastSquares.h"

// NOTE: differenza tramite prodotto di matrici omogenee
Vector3d LS::compute_error(Tricycle tricycle, Affine2d observation, Vector3d d_pose)
{
    Affine2d d_pose_sensor = v2t(tricycle.get_sensor_pose_rel()).inverse() * v2t(d_pose) * v2t(tricycle.get_sensor_pose_rel());
    return t2v(observation.inverse() * d_pose_sensor);
}

MatrixXd LS::compute_numeric_jacobian(Tricycle tricycle, 
                                      Vector3d tricycle_pose, 
                                      uint32_t actual_tick_traction,
                                      uint32_t next_tick_traction,
                                      uint32_t actual_tick_steer, 
                                      Affine2d observation,
                                      Vector3d d_pose)
{
    double epsilon = 1e-11;
    std::vector<double> parameters = tricycle.get_parameters_to_calibrate();
    MatrixXd J(3,7);
    // J = MatrixXd::Zero();
    for(int i=0; i<4; ++i){
        std::vector<double> parameters_plus = parameters;
        std::vector<double> parameters_minus = parameters;
        
        parameters_plus[i] += epsilon;
        parameters_minus[i] -= epsilon;

        Vector3d d_pose_plus = std::get<1>(tricycle.predict(parameters_plus,
                                                            tricycle_pose,
                                                            actual_tick_traction,
                                                            next_tick_traction,
                                                            actual_tick_steer));

        Vector3d d_pose_minus = std::get<1>(tricycle.predict(parameters_minus,
                                                            tricycle_pose,
                                                            actual_tick_traction,
                                                            next_tick_traction,
                                                            actual_tick_steer));

        Affine2d d_pose_sensor_plus = v2t(tricycle.get_sensor_pose_rel()).inverse() * v2t(d_pose_plus) * v2t(tricycle.get_sensor_pose_rel());
        Affine2d d_pose_sensor_minus = v2t(tricycle.get_sensor_pose_rel()).inverse() * v2t(d_pose_minus) * v2t(tricycle.get_sensor_pose_rel());

        // // ERROR: it seems to be wrong this approach
        // // NOTE: differenza matriciale tra plus e minus
        // Vector3d column = t2v(d_pose_sensor_plus.inverse() * d_pose_sensor_minus);      

        // NOTE: differenza numerica tra error plus e error minus
        // Vector3d error_plus = t2v(observation.inverse() * d_pose_sensor_plus);
        // Vector3d error_minus = t2v(observation.inverse() * d_pose_sensor_minus);
        // Vector3d column = error_plus - error_minus;
        
        // NOTE: differenza numerica
        Vector3d column = t2v(d_pose_sensor_plus) - t2v(d_pose_sensor_minus);
        
        column /= (2*epsilon);

        J.col(i) = column;                                                            
    }
    for(int i=0; i<3; ++i){
        Vector3d noise = Vector3d::Zero();
        noise[i] += epsilon;

        Affine2d sensor_pose_rel_plus = v2t(noise) * v2t(tricycle.get_sensor_pose_rel());
        Affine2d sensor_pose_rel_minus = v2t(-noise) * v2t(tricycle.get_sensor_pose_rel());
        
        Affine2d d_pose_sensor_plus = sensor_pose_rel_plus.inverse() * v2t(d_pose) * sensor_pose_rel_plus;
        Affine2d d_pose_sensor_minus = sensor_pose_rel_minus.inverse() * v2t(d_pose) * sensor_pose_rel_minus;

        // NOTE: differenza matriciale (WRONG)
        // Se fatto una sola volta sembra funzionare; se ripetuto iterativamente no
        // Vector3d column = t2v(d_pose_sensor_plus.inverse() * d_pose_sensor_minus);

        // NOTE: differenza numerica degli errori (RIGHT)
        // Vector3d error_plus = t2v(observation.inverse() * d_pose_sensor_plus);
        // Vector3d error_minus = t2v(observation.inverse() * d_pose_sensor_minus);
        // Vector3d column = error_plus - error_minus;

        // NOTE: differenza numerica (RIGHT)
        Vector3d column = t2v(d_pose_sensor_plus) - t2v(d_pose_sensor_minus);

        column /= (2*epsilon);
        J.col(i+4) = column;
    }
    return J;

}

std::vector<double> LS::calibrate_parameters(VectorXd dx, std::vector<double> parameters)
{
    std::vector<double> adjusted_parameters = parameters;
    Vector3d d_sensor = Vector3d(dx[4], dx[5], dx[6]);
    Vector3d sensor_pose_rel = Vector3d(parameters[4], parameters[5], parameters[6]);
    for(int i=0; i<4; ++i){
        adjusted_parameters[i] = parameters[i] + dx[i];
    }
    
    Vector3d sensor_pose_adjusted = t2v( v2t(d_sensor) * v2t(sensor_pose_rel) );
    adjusted_parameters[4] = sensor_pose_adjusted[0];
    adjusted_parameters[5] = sensor_pose_adjusted[1];
    adjusted_parameters[6] = sensor_pose_adjusted[2];

    return adjusted_parameters;
}

/*
    TODO: compute_analytical_jacobian
    MatrixXd LS::compute_analytic_jacobian(Tricycle tricycle, 
                                           uint32_t actual_tick_traction, 
                                           uint32_t next_tick_traction, 
                                           uint32_t actual_tick_steer)

    La funzione deve calcolare la jacobiana della predict rispetto
    a tutti i parametri che devo calibrare.
    Visto che le equazioni ce le ho espresse in forma matriciale, 
    posso usare matlab per ottenere direttamente la forma esplicita
    e anche per calcolare la jacobiana.
    Poi confronta con calcoli manuali (almeno di una)

*/