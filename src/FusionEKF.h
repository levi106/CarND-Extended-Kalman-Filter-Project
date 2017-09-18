#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <memory>
#include <experimental/propagate_const>
#include "measurement_package.h"
#include "Eigen/Dense"

namespace ExtendedKF {
class FusionEKF {
public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
  * Get estimated values.
  */
  const Eigen::VectorXd& GetEstimate() const;

private:
  class impl;
  std::experimental::propagate_const<std::unique_ptr<impl>> pImpl;
};
}

#endif /* FusionEKF_H_ */
