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
	// // Transform<float, 3, Affine> t;
	// // t = Translation<float, 3>(Vector3f(1.f, 1.f, 0.f));
	// // t.rotate(AngleAxis<float>(M_PI_2, Vector3f::UnitZ()));


	Transform<float, 3, Affine> b;
	b = Translation<float, 3>(Vector3f(1.f, 0.f, 0.f));
	b.rotate(AngleAxis<float>(M_PI_2, Vector3f::UnitZ()));

	// Take initial information to create tricycle
	// double timestamp = dataset.time[80];
	// uint32_t tick_s = dataset.ticks_steer[80];
	// uint32_t tick_t = dataset.ticks_traction[80];
	// Vector3f model_pose(dataset.model_pose_x[80], dataset.model_pose_y[80], 0.f);
	// float theta(dataset.model_pose_theta[80]);

	// Take initial information to create tricycle
	double timestamp = 1668091584.821040869;
	uint32_t tick_s = 290;
	uint32_t tick_t = 4294859756;
	Vector3d model_pose(0.d, 0.d, 0.d);
	double theta(0.d);

	// std::cout << tick_t << "\n";
	// Create tricycle with starting information
	Tricycle tricycle = Tricycle(timestamp, 
								tick_s, 
								tick_t, 
								model_pose,
								theta);

	// uint32_t next_tick = 526;
	// tricycle.step(next_tick, tick_s);

		
	for(int i=0; i<dataset.time.size()-1; ++i){
		uint32_t actual_tick_steer = dataset.ticks_steer[i];
		uint32_t next_tick_t = dataset.ticks_traction[i+1];
		tricycle.step(next_tick_t, actual_tick_steer);
	}
}
