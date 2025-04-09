# Tricycle calibration using least-squares approach
This repo contains a calibration engine for a front-traction tricycle equipped with encoders and a sensor. 

### Calibrated parameters
Parameters must be calibrated are the kinematic parameters of the robot and 
sensor pose (position and orientation) with respect to kinematic center of the robot.
In detail:
- `k_steer`: how many radians correspond to one tick;
- `k_traction`: how many meters correspond to one tick;
- `steer_offset`: at which angle corresponds the zero of the wheel;
-	`base_line`: the lenght of the base_line;
-	`sensor_pose_rel`: sensor pose $$(x, y, &theta;)$$ relative to the robot.

## Implementation
All information about implementation and mathematical formalism of the calibration engine are published at the
following [GitHub Page](https://flaviofoxes.github.io/robot-calibration-project/)

## Structure overview
In `dataset` folder there is the text file containing the dataset used to calibrated the robot.

Folder `include` contains all the header files.

Folder `src` contains all the cpp files.

In `trajectories` folder uncalibrated and calibrated trajectories are saved (it already contains some
trajectories). When the program is executed, files are overwritten.

File `view_dataset_traj.py` shows the model pose and the tracker pose in the dataset.

File `view_robot_uncalibrated_traj.py` shows the model pose (in dataset) and the uncalibrated robot pose.

File `view_sensor_uncalibrated_traj.py` shows the tracker pose (in dataset) and the uncalibrated sensor pose.

File `view_sensor_calibrated_traj.py` shows the tracker pose (in dataset) and the calibrated sensor pose.

## Installation
### Prerequities
To execute the calibration engine:
- Eigen

To visualize uncalibrated and calibrated trajectories using Python scripts:
- numpy
- matplotlib

### Build
To build the code, move in the project directory and use the MakeFile:
```
cd /path/to/robot-calibration-project
make
```

## Results
The Gauss-Newton method is run 7 times. In the first cycle, the entire dataset is used for the calibration.
Then, a sumbsampling of the dataset is actuated, putting a threshold for the error norm equals to the mean of the total
error accumulated in the previous cycle. \
In the end, the calibrated parameters are:
- `k_steer`: 0.556352;
- `k_traction`: 0.00947317;
- `steer_offset`: -0.0506322;
-	`base_line`: 1.35205;
-	`sensor_pose_rel`: (1.58809, 0.0039371, 0.0163697).

The calibrated, uncalibrated and ground truth trajectories are shown below:

<table>
  <tr>
    <td><img src="https://github.com/FlavioFoxes/robot-calibration-project/blob/main/assets/dataset.png" alt="Dataset"><br><strong>Dataset model pose - Dataset tracker pose</strong></td>
    <td><img src="https://github.com/FlavioFoxes/robot-calibration-project/blob/main/assets/uncalibrated_robot.png" alt="Uncalibrated Robot"><br><strong>Dataset model pose - Uncalibrated robot pose</strong></td>
  </tr>
  <tr>
    <td><img src="https://github.com/FlavioFoxes/robot-calibration-project/blob/main/assets/uncalibrated_sensor.png" alt="Uncalibrated Sensor"><br><strong>Dataset tracker pose - Uncalibrated sensor pose</strong></td>
    <td><img src="https://github.com/FlavioFoxes/robot-calibration-project/blob/main/assets/calibrated_sensor.png" alt="Calibrated Sensor"><br><strong>Dataset tracker pose - Calibrated sensor pose</strong></td>
  </tr>
</table>
