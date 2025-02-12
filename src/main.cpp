#include <iostream>
#include <Eigen/Geometry>
#include <cmath> 
#include <cfloat>
#include "utils.h"
#include "tricycle.h"

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

	// // Take initial information to create tricycle
	// double timestamp = 1668091584.821040869;
	// uint32_t tick_s = 290;
	// uint32_t tick_t = 4294859756;
	// Vector3d model_pose(0.d, 0.d, 0.d);

	// Create tricycle with starting information
	Tricycle tricycle = Tricycle(timestamp,
								 tick_s,
								 tick_t,
								 model_pose);

	// Simulation loop

	// TODO: add H matrix where to sum J^T * J
	//	 	 add b matrix where to sum J^T * e
	
	// For each measurement
	for(int i=0; i<dataset.time.size()-1; ++i){
		// Take encoder measurements
		uint32_t actual_tick_steer = dataset.ticks_steer[i];
		uint32_t next_tick_t = dataset.ticks_traction[i+1];
		// Move tricycle of one step
		tricycle.step(next_tick_t, actual_tick_steer);
		
		// Compute error between tricycle sensor pose and tracker pose
		// Vector3d error = LS::compute_error(tricycle, 
		// 								   Vector3d(dataset.tracker_pose_x[i], dataset.tracker_pose_y[i], dataset.tracker_pose_theta[i]));
		// LS::compute_numeric_jacobian(tricycle, )


	}
	
}
