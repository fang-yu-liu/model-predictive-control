# Autonomous Driving using MPC (Model Predictive Control)
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This project is part of the [Udacity Self-Driving Car Nanodegree](https://www.udacity.com/drive) program, and some of the code are leveraged from the lecture materials.

---

[//]: # (Image References)

[image1]: ./write-up-data/model_update_equations.png "Model Update Equations"

## Objective

This project uses a Model Predictive Control (MPC) to drive the vehicle in a simulator. A set of reference waypoints is provided by the simulator, and we use MPC to compute the optimal trajectory with associated steering angle and throttle commands to minimize the error between the calculated trajectory and the third-order polynomial fitted from the given waypoints.

## Overview
#### Kinematic Model
* State - vehicle's x and y coordinates, orientation angle (psi), velocity (v), cross track error (cte) and orientation error (epsi).
* Actuators - steering angle (delta) and throttle (a).

![Update Equations][image1]

#### Timestep Length and Elapsed Duration (N & dt)
The prediction horizon (T) is the duration over which future predictions are made. T is the product of two other variables, T = N * dt.
* N - The number of timesteps in the horizon. In the case of driving a car, T should be a few seconds, at most. Beyond that horizon, the environment will change enough that it won't make sense to predict any further into the future.
* dt - how much time elapses between actuations.

The final value chosen for N and dt are 10 and 0.1. I first aimed T around 1s and then slowly tuned N and dt. Tried several values and 10 and 0.1 seemed to produce the smoothest driving experience.

#### Waypoints Preprocessing
The waypoints given by the simulator are transformed from global coordinate to the vehicle coordinate (main.cpp line 101 - line 107) before sending to MPC.

#### Dealing with latency
The update equations above use the actuator values from previous timestep. However, if we account for the latency around 100ms and duration for each timestep is also around 100ms, then the equations should depend on the actuator values from two steps before. (See updated equations in MPC.cpp line 98 - line 102)

#### Cost Function Weighting
For the desired velocity, I first tested with ref_v = 20, and the car drove smoothly around the track. I then tried to increase the velocity by 10 each time. The car started driving out of the line around corners under high speed. Therefore, I slightly tuned the cost function weighting, increased the weight for the terms that account for cross track error, orientation error and gap between sequential actuations to make sure it can still follow the trajectory smoothly under high speed. (MPC.cpp line 48 - line 65)

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
