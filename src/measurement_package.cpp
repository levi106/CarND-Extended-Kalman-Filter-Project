#include "measurement_package.h"

std::istream& operator>> (std::istream &is, MeasurementPackage &measurement_pack) {
  std::string sensor_type;
  is >> sensor_type;
  if (sensor_type.compare("L") == 0) {
    measurement_pack.sensor_type_ = MeasurementPackage::LASER;
    measurement_pack.raw_measurements_ = Eigen::VectorXd(2);
    float px, py;
    is >> px;
    is >> py;
    measurement_pack.raw_measurements_ << px, py;
    is >> measurement_pack.timestamp_;
    measurement_pack.is_valid_ = true;
  } else if (sensor_type.compare("R") == 0) {
    measurement_pack.sensor_type_ = MeasurementPackage::RADAR;
    measurement_pack.raw_measurements_ = Eigen::VectorXd(3);
    float ro, theta, ro_dot;
    is >> ro;
    is >> theta;
    is >> ro_dot;
    measurement_pack.raw_measurements_ << ro, theta, ro_dot;
    is >> measurement_pack.timestamp_;
    measurement_pack.is_valid_ = true;
  }
  return is;
}
