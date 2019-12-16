# **Self-Driving Car**
# **Project: Extended Kalman Filter**

## MK

Overview

Implement an Extened Kalman Filter in C++. Using simulated lidar and radar measurements of the bicycle, that travels around the car, track the bicycle's position and velocity using Kalman filter, lidar measurements, and radar measurements.

#
### The Project

[//]: # (Image References)

[image1]: ./Writeup_IV/DataFile_ScreenShot.png "DataFile_ScreenShot"
[image2]: ./Writeup_IV/.png ""

#
Data File for EKF project

Explanation of the Data File

The github repo contains one data file:

- `obj_pose-laser-radar-synthetic-input.txt`

A screenshot of the first data file:

The simulator will be using this data file, and feed `main.cpp` values from it one line at a time.

![][image1]

Each row represents a sensor measurement, where the first column lists source of measurement: radar (R) or lidar (L).

For a row containing radar data, the columns are: `sensor_type`, `rho_measured`, `phi_measured`, `rhodot_measured`, `timestamp`, `x_groundtruth`, `y_groundtruth`, `vx_groundtruth`, `vy_groundtruth`, `yaw_groundtruth`, `yawrate_groundtruth`.

For a row containing lidar data, the columns are: `sensor_type`, `x_measured`, `y_measured`, `timestamp`, `x_groundtruth`, `y_groundtruth`, `vx_groundtruth`, `vy_groundtruth`, `yaw_groundtruth`, `yawrate_groundtruth`.

Whereas radar has three measurements `(rho, phi, rhodot)`, lidar has two measurements `(x, y)`.

Use measurement values and timestamp inside Kalman filter algorithm. Groundtruth, which represents the actual path the bicycle took, is for calculating `root mean squared error (RMSE)`.

Ignore yaw and yaw rate ground truth values.


Reading in the Data

A wrapper code was provided to read in and parse the data. This code is in the `main.cpp` file. The `main.cpp` file creates instances of a `MeasurementPackage`.

Inside `main.cpp`, wrapper code would be something similar to as shown below:
```
MeasurementPackage meas_package;
meas_package.sensor_type_ = MeasurementPackage::LASER;
meas_package.raw_measurements_ = VectorXd(2);
meas_package.raw_measurements_ << px, py;
meas_package.timestamp_ = timestamp;
```
and
```
vector<VectorXd> ground_truth;
VectorXd gt_values(4);
gt_values(0) = x_gt;
gt_values(1) = y_gt; 
gt_values(2) = vx_gt;
gt_values(3) = vy_gt;
ground_truth.push_back(gt_values);
```
The code reads in the data file line by line. The measurement data for each line gets pushed onto a `measurement_pack_list`. The ground truth `[p_x, p_y, v_x, v_y]` for each line in the data file gets pushed onto `ground_truth` so RMSE can be calculated later from `tools.cpp`.

#
File Structure

Overview of a Kalman Filter: Initialize, Predict, Update

Three main steps for programming a Kalman filter:
- `Initialize` Kalman filter variables
- `Predict` object location/position after a time step of Δt
- `Update` object location/position based on sensor measurements

The prediction and update steps are called recursively in a loop

To measure how well Kalman filter performs, `RMSE` is calculated by comparing Kalman filter results with the ground truth.

The three steps (initialize, predict, update) plus calculating RMSE encapsulate the entire extended Kalman filter project.
