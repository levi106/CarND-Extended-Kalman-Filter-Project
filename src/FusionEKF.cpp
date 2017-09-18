#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "tools.h"

namespace ExtendedKF {

class FusionEKF::impl {
public:
  impl();
  ~impl();

  void ProcessMeasurement(const MeasurementPackage &measurement_pack);
  const Eigen::VectorXd& GetEstimate() const;

private:
  void Init(const MeasurementPackage &measurement_pack);
  void InitWithRadarData(const MeasurementPackage &measurement_pack);
  void InitWithLaserData(const MeasurementPackage &measurement_pack);

  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // Kalman Filter update and prediction math lives in here.
  KalmanFilter ekf_;

  // tool object used to compute Jacobian and RMSE
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
};

FusionEKF::FusionEKF() : pImpl{std::make_unique<impl>()} {
}

FusionEKF::~FusionEKF() = default;

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  pImpl->ProcessMeasurement(measurement_pack);
}

const Eigen::VectorXd& FusionEKF::GetEstimate() const {
  return pImpl->GetEstimate();
}

FusionEKF::impl::impl() 
  : is_initialized_{false}
  , previous_timestamp_{0}
  , R_laser_{Eigen::MatrixXd{2,2}}
  , R_radar_{Eigen::MatrixXd{3,3}}
  , H_laser_{Eigen::MatrixXd{2,4}} {

  // measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;
  
  // measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;            

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
}

FusionEKF::impl::~impl() = default;

void FusionEKF::impl::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    Init(measurement_pack);
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.SetPredictMatrices(dt);
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Eigen::MatrixXd Hj = Tools::CalculateJacobian(ekf_.get_x());
    ekf_.SetUpdateMatrices(R_radar_, Hj);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.SetUpdateMatrices(R_laser_, H_laser_);
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  std::cout << "x_ = " << ekf_.get_x() << std::endl;
  std::cout << "P_ = " << ekf_.get_P() << std::endl;
}

const Eigen::VectorXd& FusionEKF::impl::GetEstimate() const {
  return ekf_.get_x();
}

void FusionEKF::impl::Init(const MeasurementPackage &measurement_pack) {
  /**
  TODO:
    * Initialize the state ekf_.x_ with the first measurement.
    * Create the covariance matrix.
    * Remember: you'll need to convert radar from polar to cartesian coordinates.
  */
  // first measurement
 
  std::cout << "EKF: " << std::endl;

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    InitWithRadarData(measurement_pack);
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    InitWithLaserData(measurement_pack);
  }

  // done initializing, no need to predict or update
  is_initialized_ = true;
}

void FusionEKF::impl::InitWithRadarData(const MeasurementPackage &measurement_pack) {
  // Convert radar from polar to cartesian coordinates and initialize state.
  float rho = measurement_pack.raw_measurements_[0];
  float phi = measurement_pack.raw_measurements_[1];
  float x = rho * std::cos(phi);
  float y = rho * std::sin(phi);
  ekf_.set_x(x, y, 0, 0);
  previous_timestamp_ = measurement_pack.timestamp_;
}

void FusionEKF::impl::InitWithLaserData(const MeasurementPackage &measurement_pack) {
  // Initialize state.
  ekf_.set_x(measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0);  
  previous_timestamp_ = measurement_pack.timestamp_;
}

}
