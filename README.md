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


## Project Instructions and Rubric

![](https://github.com/emilkaram/Self-Driving-Cars-extended-kalman-filter-c-Project-udacity--Term2--Project1/blob/master/img/1.png)


 
## Hints and Tips!

 
