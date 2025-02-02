#include <iostream>
#include <Eigen/Geometry>
#include <cmath> 
#include "utils.h"
#include "tricycle.h"

using Eigen::MatrixXd;
using namespace Eigen;

int main(){


	MatrixXd m(2,2);
	m(0,0) = 3;
	m(1,0) = 2.5;
	m(0,1) = -1;
	m(1,1) = m(1,0) + m(0,1);
	// std::cout << m << std::endl;
		
	// Transformation matrix
	Transform<float, 3, Affine> t;
	t = Translation<float, 3>(Vector3f(1.f, 1.f, 0.f));
	t.rotate(AngleAxis<float>(M_PI_2, Vector3f::UnitZ()));


	// Create dataset
	utils::Dataset dataset = utils::create_dataset(std::string("dataset/dataset.txt"));

	
	// Take initial information to create tricycle
	double timestamp = dataset.time[0];
	uint32_t tick_s = dataset.ticks_steer[0];
	uint32_t tick_t = dataset.ticks_traction[0];
	Vector3f model_pose(dataset.model_pose_x[0], dataset.model_pose_y[0], 0.f);
	float theta(dataset.model_pose_theta[0]);

	// Create tricycle with starting information
	Tricycle tricycle = Tricycle(timestamp, 
								tick_s, 
								tick_t, 
								model_pose,
								theta);

	tricycle.step(timestamp, tick_t);
	
}