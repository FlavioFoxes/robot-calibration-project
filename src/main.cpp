#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Cholesky> 
#include <cmath> 
#include <cfloat>
#include "utils.h"
#include "leastSquares.h"
// #include "tricycle.h"

using Eigen::MatrixXd;
using namespace Eigen;

int main(){


	// --------------------------
	// 			  MAIN
	// --------------------------

	// Create dataset from dataset file
	utils::Dataset dataset = utils::create_dataset(std::string("dataset/dataset.txt"));

	// Take initial information to create tricycle
	double timestamp = dataset.time[0];
	uint32_t tick_s = dataset.ticks_steer[0];
	uint32_t tick_t = dataset.ticks_traction[0];
	Vector3d model_pose(dataset.model_pose_x[0], dataset.model_pose_y[0], dataset.model_pose_theta[0]);

	// Create tricycle with starting information
	Tricycle tricycle = Tricycle(timestamp,
								 tick_s,
								 tick_t,
								 model_pose);

	for(int j=0; j<30; ++j){

		
		// Initialize H and b matrices
		MatrixXd H(7,7);
		H.setZero();
		VectorXd b(7);
		b.setZero();

		double chi_sum = 0.0;
		// Simulation loop
		// For each measurement
		for(int i=0; i<dataset.time.size()-1; ++i){
			// Take encoder measurements
			uint32_t actual_tick_steer = dataset.ticks_steer[i];
			uint32_t actual_tick_traction = dataset.ticks_traction[i];
			uint32_t next_tick_traction = dataset.ticks_traction[i+1];
			// Actual tricycle pose (before applying the step)
			// It is useful for error and jacobian computations
			Vector3d tricycle_pose = tricycle.get_pose();
			
			tricycle.step(actual_tick_traction, next_tick_traction, actual_tick_steer, false);
			
			/*
			FIXME from here
			*/

			// Compute error between tricycle sensor pose and tracker pose
			Vector3d tracker_pose = Vector3d(dataset.tracker_pose_x[i+1], dataset.tracker_pose_y[i+1], dataset.tracker_pose_theta[i+1]);	
			Vector3d error = LS::compute_error(tricycle, //tricycle.get_sensor_pose());
											   tracker_pose);
			// std::cout << "----------JAC----------\n";
			
			MatrixXd J = LS::compute_numeric_jacobian(tricycle, tricycle_pose, actual_tick_traction, next_tick_traction, actual_tick_steer, tracker_pose);

			// std::cout << "Jacobian:		\n" << J << std::endl;
			// assert(0); 
			double chi = error.transpose() * error;
			if(chi > 1.0){
				error *= sqrt(1.0/chi);
			}
			
			H += (J.transpose() * J);
			b += (J.transpose() * error);
			chi_sum += chi;
			
		}
		VectorXd dx = H.ldlt().solve(-b);
		std::cout << "dx:\n" << dx << std::endl;
	
		std::vector<double> parameters = tricycle.get_parameters_to_calibrate();
		for(int i=0; i<4; ++i){
			parameters[i] += dx[i];
		}
		Vector3d sensor_pose_rel = Vector3d(parameters[4], parameters[5], parameters[6]);
		Vector3d sensor_adjustment = Vector3d(dx[4], dx[5], dx[6]);
		Vector3d sensor_calibrated = utils::t2v( utils::v2t(sensor_adjustment) * utils::v2t(sensor_pose_rel) );
		parameters[4] = sensor_calibrated[0];
		parameters[5] = sensor_calibrated[1];
		parameters[6] = sensor_calibrated[2];
		tricycle.set_calibrated_parameters(parameters);

		std::cout << "-----PARAMETERS-----\n";
		for(int i=0; i<parameters.size(); ++i){
			std::cout << parameters[i] << std::endl;
		}

	
	}
	// Solve linear system
	// VectorXd dx = H.completeOrthogonalDecomposition().pseudoInverse() * (-b);
	
	// DEBUG
	// for(int i = 0; i<7; ++i){
	// 	std::cout << dx[i] << std::endl;
	// }

	// Adjust kinematic parameters


	// Print calibrated trajectory
	tricycle.set_model_pose(model_pose);
	std::vector<double> parameters = tricycle.get_parameters_to_calibrate();
	std::cout << "----------------------"<< std::endl;
	for(int i=0; i<parameters.size(); ++i){
		std::cout << parameters[i] << std::endl;
	}

	for(int i=0; i<dataset.time.size()-1; ++i){
		// Take encoder measurements
		uint32_t actual_tick_steer = dataset.ticks_steer[i];
		uint32_t actual_tick_traction = dataset.ticks_traction[i];
		uint32_t next_tick_traction = dataset.ticks_traction[i+1];

		tricycle.step(actual_tick_traction, next_tick_traction, actual_tick_steer, true);
	}
}
