#include "tools.h"
#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>

namespace ExtendedKF {
namespace Tools {
Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                       const std::vector<Eigen::VectorXd> &ground_truth) {
  /**
   * TODO:
   * Calculate the RMSE here.
   */
  Eigen::VectorXd rmse{4};
  rmse << 0,0,0,0;

  if (estimations.size() == 0 || (estimations.size() != ground_truth.size())) {
    std::cout << "Warning: invalid estimations size" << std::endl;
    return rmse;
  }
  for (int i = 0; i < estimations.size(); ++i) {
    Eigen::VectorXd diff = estimations[i] - ground_truth[i];
    diff = diff.array() * diff.array();
    rmse += diff;
  }
  rmse /= estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  Eigen::MatrixXd Hj{3,4};

  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  float rho_2 = px*px + py*py;
  float rho = sqrt(rho_2);

  // check division by zero
  if (std::fabs(rho_2) < 0.0001) {
    std::cout << "Warning: px == py == 0" << std::endl;
    return Hj;
  }

  // compute the Jacobian matrix
  Hj << px/rho, py/rho, 0, 0,
        -py/rho_2, px/rho_2, 0, 0,
        py*(vx*py-vy*px) / (rho*rho_2), px*(vy*px-vx*py) / (rho*rho_2), px/rho, py/rho;

  return Hj;
}
};
};
