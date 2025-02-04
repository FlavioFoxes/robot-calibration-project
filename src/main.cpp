#include <iostream>
#include <Eigen/Geometry>
#include <cmath> 
#include "utils.h"
#include "tricycle.h"

using Eigen::MatrixXd;
using namespace Eigen;

int main(){

	// Transformation matrix
	Transform<float, 3, Affine> t;
	t = Translation<float, 3>(Vector3f(1.f, 0.f, 0.f));
	t.rotate(AngleAxis<float>(M_PI_2, Vector3f::UnitZ()));


	Transform<float, 3, Affine> b;
	b = Translation<float, 3>(Vector3f(1.f, 0.f, 0.f));
	b.rotate(AngleAxis<float>(M_PI_2, Vector3f::UnitZ()));

	Transform<float, 3, Affine> c;
	c = t*b;
	
	std::cout << c.matrix() << std::endl;

	// // Create dataset
	// utils::Dataset dataset = utils::create_dataset(std::string("dataset/dataset.txt"));

	
	// // Take initial information to create tricycle
	// double timestamp = dataset.time[69];
	// uint32_t tick_s = dataset.ticks_steer[69];
	// uint32_t tick_t = dataset.ticks_traction[69];
	// Vector3f model_pose(dataset.model_pose_x[69], dataset.model_pose_y[69], 0.f);
	// float theta(dataset.model_pose_theta[69]);

	// // Create tricycle with starting information
	// Tricycle tricycle = Tricycle(timestamp, 
	// 							tick_s, 
	// 							tick_t, 
	// 							model_pose,
	// 							theta);

	// std::cout << dataset.time.size() << std::endl;
	// for(int i=70; i<dataset.time.size(); ++i){
	// 	double next_tick_t = dataset.ticks_traction[i];
	// 	tricycle.step(next_tick_t);
	// }


}
