#include "Logger.h"
#include <fstream>

namespace ExtendedKF {

class Logger::impl {
public:
  impl();
  virtual ~impl();

  void Init();
  void Log(const MeasurementPackage &measurement_pack, const Eigen::VectorXd &estimate, const Eigen::VectorXd &gt_values, const Eigen::VectorXd &rmse);

private:
  static const std::string filename_;
  std::ofstream os_;
};

Logger::Logger() : pImpl{std::make_unique<impl>()} {
}

Logger::~Logger() = default;

void Logger::Init() {
  pImpl->Init();
}

void Logger::Log(const MeasurementPackage &measurement_pack, const Eigen::VectorXd &estimate, const Eigen::VectorXd &gt_values, const Eigen::VectorXd &rmse) {
  pImpl->Log(measurement_pack, estimate, gt_values, rmse);
}

const std::string Logger::impl::filename_{"ekf.csv"};

Logger::impl::impl() {
  os_.open(filename_, std::ios::app);
}

Logger::impl::~impl() {
  os_.close();
}

void Logger::impl::Init() {
  os_ << "sensor_type,timestamp,sensor_0,sensor_1,sensor_3,estimate_x,estimate_y,estimate_vx,estimate_vy,rmse_x,rmse_y,rmse_vx,rmse_vy" << std::endl;
}

void Logger::impl::Log(const MeasurementPackage &measurement_pack, const Eigen::VectorXd &estimate, const Eigen::VectorXd &gt_values, const Eigen::VectorXd &rmse) {
  os_ << measurement_pack.sensor_type_ << ",";
  os_ << measurement_pack.timestamp_ << ",";
  if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    os_ << measurement_pack.raw_measurements_(0) << ",";
    os_ << measurement_pack.raw_measurements_(1) << ",,";
  } else {
    os_ << measurement_pack.raw_measurements_(0) << ",";
    os_ << measurement_pack.raw_measurements_(1) << ",";
    os_ << measurement_pack.raw_measurements_(2) << ",";
  }
  os_ << estimate(0) << ",";
  os_ << estimate(1) << ",";
  os_ << estimate(2) << ",";
  os_ << estimate(3) << ",";
  os_ << rmse(0) << ",";
  os_ << rmse(1) << ",";
  os_ << rmse(2) << ",";
  os_ << rmse(3) << std::endl;
}

}