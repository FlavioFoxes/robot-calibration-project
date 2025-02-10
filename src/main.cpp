#include <iostream>
#include <Eigen/Geometry>
#include <cmath> 
#include "utils.h"
#include "tricycle.h"
#include <cfloat>

using Eigen::MatrixXd;
using namespace Eigen;

int main(){

	// uint32_t a = 4294959605;
	// uint32_t t = 4294859756;
	// uint32_t f = 526;

	// double result = (double) f - (double) a;
	// std::cout << f-a << std::endl;
	
	
	
	// // Transformation matrix
	// Affine2d t = Affine2d::Identity();

	// double angle = double(M_PI_2);
	// // ERRORE
    // // Aggiungiamo una traslazione
    // t.translation() = Vector2d(1.0, 2.0);
	// // t.rotation() = Rotation2D(angle);
	// t.rotate(angle);
	// std::cout << t.matrix() << std::endl;
	// std::cout << "...............\n";



	// --------------------------
	// 			  MAIN
	// --------------------------

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
	// double theta(0.d);

	// Create tricycle with starting information
	Tricycle tricycle = Tricycle(timestamp, 
								tick_s, 
								tick_t, 
								model_pose);

	for(int i=0; i<dataset.time.size()-1; ++i){
		uint32_t actual_tick_steer = dataset.ticks_steer[i];
		uint32_t next_tick_t = dataset.ticks_traction[i+1];
		tricycle.step(next_tick_t, actual_tick_steer);
	}
}
