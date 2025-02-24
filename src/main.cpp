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
using namespace utils;

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
	Vector3d initial_model_pose(dataset.model_pose_x[0], dataset.model_pose_y[0], dataset.model_pose_theta[0]);
	Vector3d initial_sensor_pose = Vector3d(dataset.tracker_pose_x[0], dataset.tracker_pose_y[0], dataset.tracker_pose_theta[0]);
	
	// Create tricycle with starting information
	Tricycle tricycle = Tricycle(timestamp,
								 tick_s,
								 tick_t,
								 initial_model_pose,
								 initial_sensor_pose);

	for(int j=0; j<5; ++j){

		// Initialize H and b matrices
		MatrixXd H(7,7);
		H.setZero();
		VectorXd b(7);
		b.setZero();

		// Simulation loop
		// For each measurement
		std::cout << "##############################\n";
		for(int i=0; i<dataset.time.size()-1; ++i){
		
			// Take encoder measurements
			uint32_t actual_tick_steer = dataset.ticks_steer[i];
			uint32_t actual_tick_traction = dataset.ticks_traction[i];
			uint32_t next_tick_traction = dataset.ticks_traction[i+1];

			// Take actual tricycle pose (before applying the step)
			// It is useful for error and jacobian computations
			Vector3d actual_tricycle_pose = tricycle.get_pose();
			Vector3d actual_sensor_pose = tricycle.get_sensor_pose();
			
			// TODO:
			// predict deve restituire solo d_pose (steering angle è inutile)

			// Compute prediction of displacement of the robot
			std::tuple<double, Vector3d> tuple = tricycle.predict(tricycle.get_parameters_to_calibrate(),
															      actual_tricycle_pose,
															      actual_tick_traction,
															      next_tick_traction, 
															      actual_tick_steer);
			// Unpack steering angle and displacement
			double steering_angle = std::get<0>(tuple);
			Vector3d d_pose = std::get<1>(tuple);

			// Apply displacement to the robot
			tricycle.step(d_pose, false);
			
			// Observation is the displacement between sensor measurements at i and i+1 timestamps
			Vector3d actual_tracker_pose = Vector3d(dataset.tracker_pose_x[i], dataset.tracker_pose_y[i], dataset.tracker_pose_theta[i]);
			Vector3d next_tracker_pose = Vector3d(dataset.tracker_pose_x[i+1], dataset.tracker_pose_y[i+1], dataset.tracker_pose_theta[i+1]);
			Affine2d observation = v2t(actual_tracker_pose).inverse() * v2t(next_tracker_pose);

			// Compute error 
			Vector3d error = LS::compute_error(tricycle, observation, d_pose);
			// Compute jacobian
			MatrixXd J = LS::compute_numeric_jacobian(tricycle, actual_tricycle_pose, actual_tick_traction, next_tick_traction, actual_tick_steer, observation, d_pose);

			// Accumulate in H and b
			H += J.transpose() * J;
			b += J.transpose() * error;
		}

		// L2 regularization of H
		double lambda = 0.5;
		H += Matrix<double, 7, 7>::Identity() * lambda;

		// Solve linear system		
		VectorXd dx = H.ldlt().solve(-b);
		std::cout << "dx:\n" << dx << std::endl;

		// Calibrate parameters
		std::vector<double> calibrated_parameters = LS::calibrate_parameters(dx, tricycle.get_parameters_to_calibrate());
		tricycle.set_calibrated_parameters(calibrated_parameters);
	
		// DEBUG
		std::cout << "PARAMETERS\n";
		for(int i = 0; i < calibrated_parameters.size(); ++i){
			std::cout << calibrated_parameters[i] << std::endl;
		}

		// Reset tricycle pose and sensor pose
		tricycle.set_model_pose(initial_model_pose);
		tricycle.set_sensor_pose(initial_sensor_pose);
	}


	// TEST
	// Once parameters are calibrated, test using same dataset
	for(int i=0; i<dataset.time.size()-1; ++i){

		// Take encoder measurements
		uint32_t actual_tick_steer = dataset.ticks_steer[i];
		uint32_t actual_tick_traction = dataset.ticks_traction[i];
		uint32_t next_tick_traction = dataset.ticks_traction[i+1];

		// Compute prediction of displacement of the robot
		std::tuple<double, Vector3d> tuple = tricycle.predict(tricycle.get_parameters_to_calibrate(),
																tricycle.get_pose(),
																actual_tick_traction,
																next_tick_traction, 
																actual_tick_steer);
		Vector3d d_pose = std::get<1>(tuple);
		// Apply displacement to the robot
		tricycle.step(d_pose, true);
	}
	
}
