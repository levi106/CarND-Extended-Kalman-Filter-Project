#include "kalman_filter.h"
#include <iostream>

namespace ExtendedKF {

class KalmanFilter::impl {
public:
  impl();
  ~impl();

  void Predict();
  void Update(const Eigen::VectorXd &z);
  void UpdateEKF(const Eigen::VectorXd &z);
  void SetPredictMatrices(float dt);
  void SetUpdateMatrices(const Eigen::MatrixXd &R, const Eigen::MatrixXd &H);
  void set_x(float px, float py, float vx, float vy);
  const Eigen::VectorXd& get_x() const;
  const Eigen::MatrixXd& get_P() const;
  
private:
  // state vector
  Eigen::VectorXd x_;
  
  // state covariance matrix
  Eigen::MatrixXd P_;
  
  // state transition matrix
  Eigen::MatrixXd F_;
  
  // process covariance matrix
  Eigen::MatrixXd Q_;
  
  // measurement matrix
  Eigen::MatrixXd H_;
  
  // measurement covariance matrix
  Eigen::MatrixXd R_;

  // identity matrix
  Eigen::MatrixXd I_;

  static const float noise_ax;
  static const float noise_ay;
};

const float KalmanFilter::impl::noise_ax = 9;
const float KalmanFilter::impl::noise_ay = 9;

KalmanFilter::KalmanFilter() : pImpl{std::make_unique<impl>()} {
}

KalmanFilter::~KalmanFilter() = default;

void KalmanFilter::Predict() {
  pImpl->Predict();
}

void KalmanFilter::Update(const Eigen::VectorXd &z) {
  pImpl->Update(z);
}

void KalmanFilter::UpdateEKF(const Eigen::VectorXd &z) {
  pImpl->UpdateEKF(z);
}

void KalmanFilter::SetPredictMatrices(float dt) {
  pImpl->SetPredictMatrices(dt);
}
void KalmanFilter::SetUpdateMatrices(const Eigen::MatrixXd &R, const Eigen::MatrixXd &H) {
  pImpl->SetUpdateMatrices(R, H);
}

void KalmanFilter::set_x(float px, float py, float vx, float vy) {
  pImpl->set_x(px, py, vx, vy);
}

const Eigen::VectorXd& KalmanFilter::get_x() const {
  return pImpl->get_x();
}

const Eigen::MatrixXd& KalmanFilter::get_P() const {
  return pImpl->get_P();
}


KalmanFilter::impl::impl() 
  : x_{Eigen::VectorXd{4}}
  , P_{Eigen::MatrixXd{4,4}}
  , F_{Eigen::MatrixXd{4,4}}
  , Q_{Eigen::MatrixXd{4,4}}
  , I_{Eigen::MatrixXd::Identity(4, 4)} {

  x_ << 1, 1, 1, 1;
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;
  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;
}

KalmanFilter::impl::~impl() = default;

void KalmanFilter::impl::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  Eigen::MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::impl::Update(const Eigen::VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  Eigen::VectorXd z_pred = H_ * x_;
  Eigen::VectorXd y = z - z_pred;
  Eigen::MatrixXd Ht = H_.transpose();
  Eigen::MatrixXd S = H_ * P_ * Ht + R_;
  Eigen::MatrixXd Si = S.inverse();
  Eigen::MatrixXd PHt = P_ * Ht;
  Eigen::MatrixXd K = PHt * Si;

  // new estimate
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_) * P_;
}

void KalmanFilter::impl::UpdateEKF(const Eigen::VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float rho_2 = x_(0)*x_(0) + x_(1)*x_(1);
  float rho = std::sqrt(rho_2);
  Eigen::VectorXd z_pred = Eigen::VectorXd(3);
  z_pred << rho, std::atan2(x_(1),x_(0)), (x_(0)*x_(2)+x_(1)*x_(3))/rho;
  Eigen::VectorXd y = z - z_pred;
  while (y(1) < -M_PI) {
    y(1) = y(1) + 2 * M_PI;
  }
  while (y(1) > M_PI) {
    y(1) = y(1) - 2 * M_PI;
  }
  Eigen::MatrixXd Ht = H_.transpose();
  Eigen::MatrixXd S = H_ * P_ * Ht + R_;
  Eigen::MatrixXd Si = S.inverse();
  Eigen::MatrixXd PHt = P_ * Ht;
  Eigen::MatrixXd K = PHt * Si;

  // new estimate
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_) * P_;
}

void KalmanFilter::impl::SetPredictMatrices(float dt) {
  // Modify the F matrix so that the time is integrated
  F_(0,2) = dt;
  F_(1,3) = dt;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  // set the process covariance matrix Q
  Q_ << dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
        0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
        dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
        0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
}

void KalmanFilter::impl::SetUpdateMatrices(const Eigen::MatrixXd &R, const Eigen::MatrixXd &H) {
  R_ = R;
  H_ = H;
}

void KalmanFilter::impl::set_x(float px, float py, float vx, float vy) {
  x_ << px, py, vx, vy;
}

const Eigen::VectorXd& KalmanFilter::impl::get_x() const {
  return x_;
}

const Eigen::MatrixXd& KalmanFilter::impl::get_P() const {
  return P_;
}

}
