# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program - Term2 - Project1

![](https://github.com/emilkaram/Self-Driving-Cars-extended-kalman-filter-c-Project-udacity--Term2--Project1/blob/master/img/3.png)


In this project I utilized a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Calculating the RMSE values as KPIs to measure performance. 

The project code is c++ and uses a Simulator to demonstarte the EKF.

 I used either Linux for coding and run the simulator on Windows 10 using a port forward from the VM (Oracle VM virtualBox. 
 
To compile and run by doing the following from the project top directory.
1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Note that c++ source file are: src/FusionEKF.cpp, src/FusionEKF.h, kalman_filter.cpp, kalman_filter.h, tools.cpp, tools.h, and main.cpp 

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]


## Project description:

![](https://github.com/emilkaram/Self-Driving-Cars-extended-kalman-filter-c-Project-udacity--Term2--Project1/blob/master/img/1.png)

# The input data file:

/data/obj_pose-laser-radar-synthetic-input.txt

The simulator is using this data file, and feed main.cpp values from it one line at a time.

Each row represents a sensor measurement where the first column indicates if the measurement comes from radar (R) or lidar (L).

For a row containing radar data, the columns are: sensor_type, rho_measured, phi_measured, rhodot_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.

For a row containing lidar data, the columns are: sensor_type, x_measured, y_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.

Whereas radar has three measurements (rho, phi, rhodot), lidar has two measurements (x, y).

I used the measurement values and timestamp in my Kalman filter algorithm. Groundtruth, which represents the actual path the bicycle took, is for calculating root mean squared error.

The three main steps for programming a Kalman filter:
initializing Kalman filter variables
predicting where our object is going to be after a time step \Delta{t}Δt
updating where our object is based on sensor measurements

Then the prediction and update steps repeat themselves in a loop.

To measure how well my Kalman filter performs, I calculated root mean squared error comparing the Kalman filter results with the provided ground truth.

and the results met the rubric criteria:

Dataset(1):

![](https://github.com/emilkaram/Self-Driving-Cars-extended-kalman-filter-c-Project-udacity--Term2--Project1/blob/master/img/4.png)

Dataset(2):

![](https://github.com/emilkaram/Self-Driving-Cars-extended-kalman-filter-c-Project-udacity--Term2--Project1/blob/master/img/5.png)


# FusionEKF.cpp

I initialized variables and matrices (x, F, H_laser, H_jacobian, P, etc.)

I initialized the Kalman filter position vector with the first sensor measurements

I modified the F and Q matrices prior to the prediction step based on the elapsed time between measurements

call the update step for either the lidar or radar sensor measurement.

In case of radarmesurments I converted from polar to cartesian coordinates 
   ekf_.x_ << rho * cos(phi), rho * sin(phi), rhodot * cos(phi), rhodot * sin(phi);


# KalmanFilter Class:

kalman_filter.h defines the KalmanFilter class containing the x vector as well as the P, F, Q, H and R matrices. The KalmanFilter class also contains functions for the prediction step as well as the Kalman filter update step (lidar) and extended Kalman filter update step (radar).

Because lidar uses linear equations, in the update step I used the basic Kalman filter equations. 

On the other hand, radar uses non-linear equations, so the update step involves linearizing the equations with the Jacobian matrix. 

 

 ![](https://github.com/emilkaram/Self-Driving-Cars-extended-kalman-filter-c-Project-udacity--Term2--Project1/blob/master/img/6.png)
