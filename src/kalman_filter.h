#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <memory>
#include <experimental/propagate_const>
#include "Eigen/Dense"

namespace ExtendedKF {
class KalmanFilter {
public:
  /**
  * Constructor.
  */
  KalmanFilter();

  /**
  * Destructor.
  */
  virtual ~KalmanFilter();

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
   void Predict();
   
  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);
   
  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);

  /**
   * Set state transition matrix and process covariance matrix
   * @param dt The time difference
   */
  void SetPredictMatrices(float dt);

  /**
   * Set measurement covariance matrix and measurement matrix
   * @param R The measurement covariance matrix
   * @param H The measurement matrix
   */
  void SetUpdateMatrices(const Eigen::MatrixXd &R, const Eigen::MatrixXd &H);

  /**
   * Set 4D state vector
   * @param px The position x
   * @param py The position y
   * @param vx The velocity x
   * @param vy The velocity y
   */
  void set_x(float px, float py, float vx, float vy);

  /**
   * Get 4D state vector
   */
  const Eigen::VectorXd& get_x() const;

  /**
   * Get state covariance matrix
   */
  const Eigen::MatrixXd& get_P() const;

private:
  class impl;
  std::experimental::propagate_const<std::unique_ptr<impl>> pImpl;
};
}

#endif /* KALMAN_FILTER_H_ */
