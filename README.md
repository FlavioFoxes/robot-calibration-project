# Tricycle calibration using least-squares approach
This repo contains a calibration engine for a front-traction tricycle equipped with encoders and a sensor. 

### Calibrated parameters
Parameters must be calibrated are the kinematic parameters of the robot and 
sensor pose (position and orientation) with respect to kinematic center of the robot.
In detail:
- `k_steer`: how many radians correspond to one tick;
- `k_traction`: how many meter correspond to one tick;
- `steer_offset`: at which angle correspond the zero of the wheel;
-	`base_line`: the lenght of the base_line;
-	`sensor_pose_rel`: sensor pose (x, y, &theta;) relative to the robot.

## Structure overview
In `dataset` folder there is the text file containing the dataset used to calibrated the robot. 

## Installation
### Prerequities
To execute the calibration engine:
- Eigen

To visualize uncalibrated and calibrated trajectories using Python scripts:
- numpy
- matplotlib

### Build
To 

