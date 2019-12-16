# **Self-Driving Car**
# **Project: Extended Kalman Filter**

## MK

Overview

Implement an Extened Kalman Filter in C++. Using simulated lidar and radar measurements of the bicycle, that travels around the car, track the bicycle's position and velocity using Kalman filter, lidar measurements, and radar measurements.

#
### The Project

[//]: # (Image References)

[image1]: ./Writeup_IV/DataFile_ScreenShot.png "DataFile_ScreenShot"
[image2]: ./Writeup_IV/CarND_EKF_SOD1.gif "CarND_EKF_SOD1"
[image3]: ./Writeup_IV/CarND_EKF_SOD2.gif "CarND_EKF_SOD2"

Results: Simulator Ouput

#
Dataset 1

![][image2]

#
Dataset 2

![][image3]

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

`Initializing Variables in FusionEKF.cpp`

```
/**
* TODO: Finish initializing the FusionEKF.
* TODO: Set the process and measurement noises
*/

  // state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);    
  ekf_.P_ << 1.0, 0,   0,      0,
             0,   1.0, 0,      0,
             0,   0,   1000.0, 0,
             0,   0,   0,      1000.0;
  
  //measurement covariance matrix - laser
  R_laser_ = MatrixXd(2, 2);  
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ = MatrixXd(3, 3);  
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // measurement matrix
  H_laser_ = MatrixXd(2, 4);  
  H_laser_ << 1.0, 0,   0, 0,
              0,   1.0, 0, 0;
  
  // the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;
  
  // initializing Jacobian
  Hj_ = MatrixXd(3, 4);

```
Every time `main.cpp` calls `fusionEKF.ProcessMeasurement(measurement_pack_list[k])`, the code in `FusionEKF.cpp` will run. - If this is the first measurement, the Kalman filter will try to initialize the object's location with the sensor measurement.

`Initializing the Kalman Filter in FusionEKF.cpp`
```
void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      double rho = measurement_pack.raw_measurements_[0]; // range
      double phi = measurement_pack.raw_measurements_[1]; // bearing
      double rho_dot = measurement_pack.raw_measurements_[2]; // range rate
      // convert from polar to cartesian, px
      float x = rho * cos(phi);
      // check value of x to avoid division by zero in Jacobian
      if ( x < 0.0001 ) {x = 0.0001;}
      // convert from polar to cartesian, py
      float y =  rho * sin(phi);
      // check value of y to avoid division by zero in Jacobian
      if ( y < 0.0001  ) {y = 0.0001;}
      // convert from polar to cartesian, vx
      float vx = rho_dot * cos(phi);
      // convert from polar to cartesian, vy      
      float vy = rho_dot * sin(phi);
      ekf_.x_ << x, y, vx , vy;
     }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      // set the state with the initial location and zero velocity
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    previous_timestamp_ = measurement_pack.timestamp_;
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

```
`Predict and Update Steps in FusionEKF.cpp`

Once the Kalman filter gets initialized, the next iterations of the for loop will call the `ProcessMeasurement()` function to do the predict and update steps.
```
/**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
   // compute the time elapsed between the current and previous measurements
   // dt - expressed in seconds
   float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
   previous_timestamp_ = measurement_pack.timestamp_;
  
   // TODO: YOUR CODE HERE
   float dt_2 = dt * dt;
   float dt_3 = dt_2 * dt;
   float dt_4 = dt_3 * dt;

   // Modify the F matrix so that the time is integrated
   ekf_.F_(0, 2) = dt;
   ekf_.F_(1, 3) = dt;

   // set the process covariance matrix Q
   // set the acceleration noise components
   float noise_ax = 9.0;
   float noise_ay = 9.0;  
   ekf_.Q_ = MatrixXd(4, 4);
   ekf_.Q_ <<  dt_4/4.0*noise_ax,    0,                 dt_3/2.0*noise_ax, 0,
               0,                    dt_4/4.0*noise_ay, 0,                 dt_3/2.0*noise_ay,
               dt_3/2.0*noise_ax,    0,                 dt_2*noise_ax,     0,
               0,                    dt_3/2.0*noise_ay, 0,                 dt_2*noise_ay;
  
   // predict
   ekf_.Predict();
  ```
  
  ```
  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

   if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
     // TODO: Radar updates
     ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
     ekf_.R_ = R_radar_;
     // measurement update
     ekf_.UpdateEKF(measurement_pack.raw_measurements_);
     } 
   else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
     // TODO: Laser updates
     ekf_.H_ = H_laser_;
     ekf_.R_ = R_laser_;
     // measurement update
     ekf_.Update(measurement_pack.raw_measurements_);
  }

```
In `FusionEKF.cpp`, references were made to a variable called ekf_. The `ekf_` variable is an instance of the `KalmanFilter` class. Used `ekf_` to store Kalman filter variables `(x, P, F, H, R, Q)` and call the predict and update functions.

#
`KalmanFilter Class`

`kalman_filter.h` defines the `KalmanFilter` class containing the `x vector` as well as the `P, F, Q, H` and `R` matrices. The KalmanFilter class also contains functions for the prediction step as well as the Kalman filter update step (lidar) and extended Kalman filter update step (radar).

Added code to `kalman_filter.cpp` to implement the prediction and update equations.

Lidar uses linear equations, therefore the update step used the basic Kalman filter equations. Radar uses non-linear equations, and consequently the update step consisted of linearizing the equations with the Jacobian matrix. The `Update` function will use the standard Kalman filter equations. The `UpdateEKF` will use the extended Kalman filter equations:

```
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;  
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt(px*px + py*py);
  float theta = atan2(py, px);
  float rho_dot = (px*vx + py*vy) / rho;
  VectorXd h = VectorXd(3);
  h << rho, theta, rho_dot;
  VectorXd y = z - h;
  // normalize bearing in y vector, until -pi<bearing<+pi
  while ( y(1) > M_PI || y(1) < -M_PI ) {
     if ( y(1) > M_PI ) { y(1) -= 2 * M_PI;} 
     else if ( y(1) < M_PI ) {y(1) += 2 * M_PI;}
  }
  
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;  

}

```

#
`Tools.cpp`

This file is relatively straight forward. Implemented functions to calculate root mean squared error and the Jacobian matrix:
```
#include "tools.h"
#include <iostream>

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
   VectorXd rmse(4);
   rmse << 0.0, 0.0, 0.0, 0.0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
  
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  
   MatrixXd Hj(3,4);
   Hj << 0,0,0,0,
         0,0,0,0,
         0,0,0,0;
  
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  // check division by zero
  if (fabs(c1) < 0.0001) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  // compute the Jacobian matrix
  Hj << (px/c2),               (py/c2),               0,     0,
       -(py/c1),               (px/c1),               0,     0,
        py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}

```
