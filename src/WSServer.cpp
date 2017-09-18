#include "WSServer.h"
#include <iostream>
#include <string>
#include "uWS/uWS.h"
#include "json.hpp"
#include "FusionEKF.h"
#include "Logger.h"
#include "tools.h"

namespace ExtendedKF {

class WSServer::impl {
public:
  impl();
  ~impl();
  void Init();
  void Run();

private:
  std::string HasData(std::string s);
  void OnMessage(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode);
  void OnHttpRequest(uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t);
  void OnConnection(uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req);
  void OnDisconnection(uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length);
  Eigen::VectorXd GetGroundTruth(std::istream &is);
  Eigen::VectorXd GetEstimate(const MeasurementPackage &measurement_pack);
  std::string GetResponse(const Eigen::VectorXd &estimate, const Eigen::VectorXd &rmse);

  const int port_;
  uWS::Hub hub_;
  FusionEKF fusionEKF_;
  std::vector<Eigen::VectorXd> estimations_;
  std::vector<Eigen::VectorXd> ground_truth_;
  Logger logger_;
};

WSServer::WSServer() : pImpl{std::make_unique<impl>()} { 
}

WSServer::~WSServer() = default;

void WSServer::Init() {
  pImpl->Init();
}

void WSServer::Run() {
  pImpl->Run();
}

WSServer::impl::impl() : port_{4567} {
}

WSServer::impl::~impl() = default;

void WSServer::impl::Init() {
  hub_.onMessage([this](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    OnMessage(ws, data, length, opCode);
  });

  hub_.onHttpRequest([this](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t length, size_t remainingBytes) {
    OnHttpRequest(res, req, data, length, remainingBytes);
  });

  hub_.onConnection([this](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    OnConnection(ws, req);
  });

  hub_.onDisconnection([this](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    OnDisconnection(ws, code, message, length);
  });

  logger_.Init();
}

void WSServer::impl::Run() {
  if (hub_.listen(port_)) {
    std::cout << "Listening to port " << port_ << std::endl;
  } else {
    std::cout << "Failed to listen to port" << std::endl;
    return;
  }
  hub_.run();
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string WSServer::impl::HasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

void WSServer::impl::OnMessage(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
  // "42" at the start of the message means there's a websocket message event.
  // The 4 signifies a websocket message
  // The 2 signifies a websocket event

  if (length <= 2 || data == nullptr || data[0] != '4' || data[1] != '2') {
    return;
  }

  auto s = HasData(std::string(data));
  if (s == "") {
    std::string msg = "42[\"manual\",{}]";
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT); 
    return;  
  }

  auto jsonData = nlohmann::json::parse(s);
  std::string event = jsonData[0].get<std::string>();
  if (event != "telemetry") {
    return;
  }

  // jsonData[1] is the data JSON object
  std::string sensor_measurement = jsonData[1]["sensor_measurement"];
  std::istringstream iss(sensor_measurement);

  MeasurementPackage measurement_pack;
  iss >> measurement_pack;

  if (!measurement_pack.is_valid_) {
    std::cout << "Warning: invalid sensor measurement data" << std::endl;
    return;
  }

  Eigen::VectorXd gt_values = GetGroundTruth(iss);
  ground_truth_.push_back(gt_values);

  Eigen::VectorXd estimate = GetEstimate(measurement_pack);
  estimations_.push_back(estimate);

  Eigen::VectorXd rmse = ExtendedKF::Tools::CalculateRMSE(estimations_, ground_truth_);
  logger_.Log(measurement_pack, estimate, gt_values, rmse);

  std::string msg = GetResponse(estimate, rmse);
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

void WSServer::impl::OnHttpRequest(uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  const std::string s = "<h1>Hello world!</h1>";
  if (req.getUrl().valueLength == 1) {
    res->end(s.data(), s.length());
  } else {
    // I guess this should be done more gracefully?
    res->end(nullptr, 0);
  }
}

void WSServer::impl::OnConnection(uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
  std::cout << "Connected!!!" << std::endl;
}

void WSServer::impl::OnDisconnection(uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
  ws.close();
  std::cout << "Disconnected" << std::endl;
}

Eigen::VectorXd WSServer::impl::GetGroundTruth(std::istream &is) {
  float x_gt, y_gt, vx_gt, vy_gt;
  is >> x_gt;
  is >> y_gt;
  is >> vx_gt;
  is >> vy_gt;
  Eigen::VectorXd gt_values(4);
  gt_values(0) = x_gt;
  gt_values(1) = y_gt;
  gt_values(2) = vx_gt;
  gt_values(3) = vy_gt;
  return gt_values;
}

Eigen::VectorXd WSServer::impl::GetEstimate(const MeasurementPackage &measurement_pack) {
  // Call ProcessMeasurement for Kalman filter
  fusionEKF_.ProcessMeasurement(measurement_pack);
  return fusionEKF_.GetEstimate();
}

std::string WSServer::impl::GetResponse(const Eigen::VectorXd &estimate, const Eigen::VectorXd &rmse) {
  nlohmann::json msgJson;
  msgJson["estimate_x"] = estimate(0);
  msgJson["estimate_y"] = estimate(1);
  msgJson["rmse_x"] = rmse(0);
  msgJson["rmse_y"] = rmse(1);
  msgJson["rmse_vx"] = rmse(2);
  msgJson["rmse_vy"] = rmse(3);
  std::string msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
  return msg;
}
}
