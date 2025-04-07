---
layout: default
---

# Implementation

In this section, you can find an overview about the implementation and the results of the calibration engine.

## Data
The dataset contains 2434 measurements, where each measurement is composed by:
- $$time$$: $$\space$$ timestamp of the actual measurements;
- $$tick\_steer$$: $$\space$$ reading tick of the absolute encoder for the steering angle of the front wheel;
- $$tick\_traction$$: $$\space$$ reading tick of the incremental encoder for the traction of the front wheel;
- $$model\_pose$$: $$\space$$ [$$x_m$$,$$y_m$$,$$\theta_m$$] 3x1 vector representing the odometry of the robot (2D translation + orientation);
- $$tracker\_pose$$: $$\space$$ [$$x_t$$,$$y_t$$,$$\theta_t$$] 3x1 vector representing the ground truth 2D pose of the sensor (2D translation + orientation);

## Parameters to calibrate
Parameters must be calibrated are the kinematic parameters of the robot and 
sensor pose (position and orientation) with respect to kinematic center of the robot.
In detail:
- $$k_s$$: $$\space$$ how many radians correspond to one tick;
- $$k_t$$: $$\space$$ how many meters correspond to one tick;
- $$\delta_{offset}$$: $$\space$$ at which angle corresponds the zero of the wheel;
- $$a$$: $$\space$$ the lenght of the baseline;
- $$v_{sensor}^{robot}$$: $$\space$$ sensor pose [$$x$$, $$y$$, $$\theta$$] relative to the robot.

Thus, in total the parameters to calibrate are 7.

## Kinematic model
The kinematic model of the front-traction wheel tricycle is the following:

$$
\begin{cases}
dx = s \cos(\delta)\cos(d\theta) \\
dy = s \cos(\delta)\sin(d\theta) \\
d\theta = s \frac{\sin(\delta)}{a}
\end{cases}
$$

where:
- $$\delta$$: steering angle of the front wheel
- $$s$$:  distance traveled from tricycle
- $$a$$:  length of the baseline

The kinematic model of the robot is expressed with respect to its kinematic center. 

## Algorithm
Calibration cycle is executed multiple times. Each cycle is composed by the following steps. 

### Initialization step
Tricycle and sensor poses are set equal to $$model\_pose$$ and $$tracker\_pose$$ of the first measurement.
Also the ticks about steering angle and traction are initialized with  $$tick\_steer$$ and $$tick\_traction$$
of the first measurement. \
Finally, given that the number of parameters to calibrate is 7, two matrices $$\textbf{H}$$ (7x7 matrix) and $$\textbf{b}$$ (7x1 matrix) 
are initialized to zero, and also a variable $$total\_error$$ is initialized to zero.

### Update step
Given the measurement $$i$$-th, the following information are saved to compute the prediciton:

$$
(tick\_steer_i, tick\_traction_i, tick\_traction_{i+1}) 
$$

Then, the following steps are executed:
1. **Prediction step**:
fistly, encoder ticks are converted in steering angle and traveled distance. Starting with $$\delta$$:

$$
\delta =  
\begin{cases} 
    \frac{2\pi}{max\_steer}\cdot(tick\_steer_i - max\_steer) \cdot k_s , & \text{if   }  tick\_steer_i > \frac{max\_steer}{2}\\
    \frac{2\pi}{max\_steer}\cdot tick\_steer_i \cdot k_s, & \text{otherwise } 
\end{cases}
$$

at which the offset must be added:

$$
\delta = \delta + \delta_{offset}
$$


2. Computation of the error




## Results

