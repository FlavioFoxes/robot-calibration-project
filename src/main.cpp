#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Cholesky> 
#include <cmath> 
#include <cfloat>
#include "utils.h"
#include "leastSquares.h"

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
	uint32_t tick_s = dataset.ticks_steer[0];
	uint32_t tick_t = dataset.ticks_traction[0];
	Vector3d initial_model_pose(dataset.model_pose_x[0], dataset.model_pose_y[0], dataset.model_pose_theta[0]);
	Vector3d initial_sensor_pose(dataset.tracker_pose_x[0], dataset.tracker_pose_y[0], dataset.tracker_pose_theta[0]);
	
	// Create tricycle with starting information
	Tricycle tricycle = Tricycle(tick_s,
								 tick_t,
								 initial_model_pose,
								 initial_sensor_pose);

	// std::vector for storing robot trajectory with uncalibrated parameters
	std::vector<Vector3d> robot_uncalibrated_trajectory{tricycle.get_pose()};
	std::vector<Vector3d> sensor_uncalibrated_trajectory{tricycle.get_sensor_pose()};

	// Initial threshold. 
	// In this way at first iteration all samples from dataset are used
	float threshold = 9999;
	
	// Calibration cycle multiple times
	for(int j=0; j<7; ++j){
		// Number of discarded samples
		int num_discarded = 0;
		// Total error over the dataset
		double total_error = 0;

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
			
			// Compute prediction of displacement of the robot
			Vector3d d_pose = tricycle.predict(tricycle.get_parameters_to_calibrate(),
											   actual_tricycle_pose,
											   actual_tick_traction,
											   next_tick_traction, 
											   actual_tick_steer);

			// Apply displacement to the robot
			tricycle.step(d_pose, false);
			// Save robot and sensor pose when parameters are not calibrated 
			if(j==0){
				robot_uncalibrated_trajectory.push_back(tricycle.get_pose());
				sensor_uncalibrated_trajectory.push_back(tricycle.get_sensor_pose());
			} 
			
			// Observation is the displacement between sensor measurements 
			// at i and i+1 timestamps (as affine transformation)
			Vector3d actual_tracker_pose = Vector3d(dataset.tracker_pose_x[i], dataset.tracker_pose_y[i], dataset.tracker_pose_theta[i]);
			Vector3d next_tracker_pose = Vector3d(dataset.tracker_pose_x[i+1], dataset.tracker_pose_y[i+1], dataset.tracker_pose_theta[i+1]);
			Affine2d observation = v2t(actual_tracker_pose).inverse() * v2t(next_tracker_pose);

			// Compute error and add it to total_error
			Vector3d error = LS::compute_error(tricycle, observation, d_pose);
			total_error += error.norm();
			// If error is higher than the mean error of the previous calibration cycle, 
			// discard the sample
			if(error.norm() > threshold){
				++num_discarded;
				continue;
			}
			// Compute jacobian
			MatrixXd J = LS::compute_numeric_jacobian(tricycle, actual_tricycle_pose, actual_tick_traction, next_tick_traction, actual_tick_steer, d_pose);

			// Accumulate in H and b
			H += J.transpose() * J;
			b += J.transpose() * error;
		}

		std::cout << "num discarded:	" << num_discarded << std::endl;

		// Threshold is the mean of the errors over the dataset
		threshold = total_error / dataset.time.size();

		// L2 regularization of H
		double lambda = 0.5;
		H += Matrix<double, 7, 7>::Identity() * lambda;

		// Solve linear system		
		VectorXd dx = H.ldlt().solve(-b);

		// Calibrate parameters
		std::vector<double> calibrated_parameters = LS::calibrate_parameters(dx, tricycle.get_parameters_to_calibrate());
		tricycle.set_calibrated_parameters(calibrated_parameters);
	
		// DEBUG
		std::cout << "dx:\n" << dx << std::endl;
		std::cout << "PARAMETERS\n";
		for(int i = 0; i < calibrated_parameters.size(); ++i){
			std::cout << calibrated_parameters[i] << std::endl;
		}

		// Reset tricycle pose and sensor pose
		tricycle.set_model_pose(initial_model_pose);
		tricycle.set_sensor_pose(initial_sensor_pose);
	}
	// Write on file robot and sensor uncalibrated trajectories
	utils::write_trajectory(std::string("trajectories/robot_trajectory_uncalibrated.txt"), robot_uncalibrated_trajectory);
	utils::write_trajectory(std::string("trajectories/sensor_trajectory_uncalibrated.txt"), sensor_uncalibrated_trajectory);

	// TEST
	// Once parameters are calibrated, test using same dataset

	// std::vector for storing robot trajectory with calibrated parameters
	std::vector<Vector3d> robot_calibrated_trajectory{tricycle.get_pose()};
	std::vector<Vector3d> sensor_calibrated_trajectory{tricycle.get_sensor_pose()};

	for(int i=0; i<dataset.time.size()-1; ++i){
		// Take encoder measurements
		uint32_t actual_tick_steer = dataset.ticks_steer[i];
		uint32_t actual_tick_traction = dataset.ticks_traction[i];
		uint32_t next_tick_traction = dataset.ticks_traction[i+1];

		// Compute prediction of displacement of the robot
		Vector3d d_pose = tricycle.predict(tricycle.get_parameters_to_calibrate(),
										   tricycle.get_pose(),
										   actual_tick_traction,
										   next_tick_traction, 
										   actual_tick_steer);
		// Apply displacement to the robot
		tricycle.step(d_pose, true);
		robot_calibrated_trajectory.push_back(tricycle.get_pose());
		sensor_calibrated_trajectory.push_back(tricycle.get_sensor_pose());
	}
	
	// Write on file robot and sensor calibrated trajectories
	utils::write_trajectory(std::string("trajectories/robot_trajectory_calibrated.txt"), robot_calibrated_trajectory);
	utils::write_trajectory(std::string("trajectories/sensor_trajectory_calibrated.txt"), sensor_calibrated_trajectory);

}
