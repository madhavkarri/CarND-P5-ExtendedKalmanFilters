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

#
Files Submitted

Project includes the following set of files:
- `main.cpp` - communicates with Simulator to receive data measurements, calls a function to run Kalman filter, calls a function to calculate RMSE
- `FusionEKF.cpp` - initializes Kalman filter, calls the predict function, calls the update function
- `kalman_filter.cpp` - defines the predict function, the update function for lidar, and the update function for radar
- `tools.cpp` - function to calculate RMSE and the Jacobian matrix

Files that require additions and modifications are: `FusionEKF.cpp`, `kalman_filter.cpp`, and `tools.cpp`.

How files relate to each other and a brief overview of code execution flow:

- `Main.cpp` reads in the data and sends a sensor measurement to `FusionEKF.cpp`
- `FusionEKF.cpp` takes the sensor data and initializes variables and updates variables. The Kalman filter equations are not in this file. `FusionEKF.cpp` has a variable called `ekf_`, which is an instance of a `KalmanFilter` class. The `ekf_` holds the matrix and vector values. In addition, use the `ekf_` instance to call the predict and update equations.
- The `KalmanFilter` class is defined in `kalman_filter.cpp` and `kalman_filter.h`. Modify only `kalman_filter.cpp`, which contains functions for the prediction and update steps.

#
`main.cpp`

The simulator is a client, and the c++ program is a web server.

`main.cpp` reads in the sensor data. `main.cpp` reads in sensor data line by line from the client and stores the data into a measurement object. Measurement object passes the data to the Kalman filter for processing. In addition, a ground truth and an estimation lists are used for RMSE tracking.

`main.cpp` is made up of several functions within main(), these all handle the `uWebsocketIO` communication between the simulator and it's self.

Below is the main protocol that `main.cpp` uses for `uWebSocketIO` in communicating with the simulator.
```
INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]
```
All the main code loops in `h.onMessage()`, to have access to intial variables that were created at the beginning of `main()`, pass pointers as arguments into the header of `h.onMessage()`.

For example:
```
h.onMessage([&fusionEKF,&tools,&estimations,&ground_truth]
            (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
             uWS::OpCode opCode)
```
The rest of the arguments in `h.onMessage` are used to set up the server.
```
// Create a Fusion EKF instance
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  //Call the EKF-based fusion
  fusionEKF.ProcessMeasurement(meas_package); 
```
The code is:
- Creating an instance of the `FusionEKF` class
- Receiving measurement data calling the `ProcessMeasurement()` function. `ProcessMeasurement()` is responsible for the initialization of the Kalman filter as well as calling the prediction and update steps of the Kalman filter. Implemented the `ProcessMeasurement()` function in `FusionEKF.cpp`:
Finally,

The remainder of `main.cpp` outputs the following results to the simulator:
- estimation position
- calculated RMSE

`main.cpp` calls a function to calculate root mean squared error:
```
// compute the accuracy (RMSE)
  Tools tools;
  cout << "Accuracy - RMSE:" << endl << tools.CalculateRMSE(estimations, ground_truth) << endl;
```
Implemented RMSE function in `tools.cpp` file.

#
`FusionEKF.cpp`

In `FusionEKF.cpp`, implemented sensor fusion. In this file, initialized variables, initialized Kalman filters, and calling functions that implement the prediction step or update step.

- Initialized variables and matrices (x, F, H_laser, H_jacobian, P, etc.)
- Initialized the Kalman filter position vector with the first sensor measurements
- Modified the F and Q matrices prior to the prediction step based on the elapsed time between measurements
- Call the update step based on whether the sensor measurement was from lidar or radar. As the update step for lidar and radar differ slightly, there are different set of functions for updating lidar and radar.
