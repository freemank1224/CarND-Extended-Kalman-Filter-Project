#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include "Dense"
#include "measurement_package.h"
#include <math.h>
#include "FusionEKF.h"
#include "tools.h"

using namespace std;

int main()
{
  /*******************************************************************************
   *  Set Measurements                               *
   *******************************************************************************/
  vector<MeasurementPackage> measurement_pack_list;

  // hardcoded input file with laser and radar measurements
  string in_file_name_ = "obj_pose-laser-radar-synthetic-input.txt";
  ifstream in_file(in_file_name_.c_str(),std::ifstream::in);

  if (!in_file.is_open()) {
    cout << "Cannot open input file: " << in_file_name_ << endl;
  }

  string line;
  // set i to get only first 3 measurments
  int i = 0;
  while(getline(in_file, line) && (i<=1000)){

    MeasurementPackage meas_package;

    istringstream iss(line);
    /////////////////////
    cout << iss << endl;
    /////////////////////
    string sensor_type;
    iss >> sensor_type; //reads first element from the current line
    int64_t timestamp;
    if(sensor_type.compare("L") == 0){  //laser measurement
      //read measurements
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      float x;
      float y;
      iss >> x;
      iss >> y;
      meas_package.raw_measurements_ << x,y;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);

    }else if(sensor_type.compare("R") == 0){
      //Skip Radar measurements
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      float rho;
      float phi;
      float rho_dot;
      iss >> rho;
      iss >> phi;
      iss >> rho_dot;
      meas_package.raw_measurements_ << rho, phi, rho_dot;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);

      //continue;
    }

    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;
    VectorXd gt_values(4);
    gt_values(0) = x_gt;
    gt_values(1) = y_gt; 
    gt_values(2) = vx_gt;
    gt_values(3) = vy_gt;
    ground_truth.push_back(gt_values);


    i++;  //move to the next line

  }

  // Create a Kalman Filter instance 
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  //call the ProcessingMeasurement() function for each measurement
  size_t N = measurement_pack_list.size();
  for (size_t k = 0; k < N; ++k) {  //start filtering from the second frame (the speed is unknown in the first frame)
    FusionEKF.ProcessMeasurement(measurement_pack_list[k]);
        

    //Push the current estimated x,y positon from the Kalman filter's state vector
    VectorXd estimate(4);

    double p_x = fusionEKF.ekf_.x_(0);
    double p_y = fusionEKF.ekf_.x_(1);
    double v1  = fusionEKF.ekf_.x_(2);
    double v2 = fusionEKF.ekf_.x_(3);

    estimate(0) = p_x;
    estimate(1) = p_y;
    estimate(2) = v1;
    estimate(3) = v2;
    
    estimations.push_back(estimate);

    VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
  }

  if(in_file.is_open()){
    in_file.close();
  }
  return 0;



}
