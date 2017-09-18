#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include <iostream>
#include "Eigen/Dense"

class MeasurementPackage {
public:
  long long timestamp_;

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;

  bool is_valid_;

  MeasurementPackage() : is_valid_(false) {};
};

std::istream& operator>> (std::istream &is, MeasurementPackage &measurement_pack);

#endif /* MEASUREMENT_PACKAGE_H_ */
