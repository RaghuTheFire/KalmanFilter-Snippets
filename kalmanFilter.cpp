
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
using namespace Eigen;

class KalmanFilter 
{
  public:
      KalmanFilter(int state_dim = 3, double time_resolution = 1.0) 
      {
          n = state_dim;
          dt = time_resolution;
  
          // Define the state transition matrix F
          F = Eigen::MatrixXd::Identity(n, n);
          for (int i = 1; i < n; i++) 
          {
              F(i - 1, i) = dt;
          }
          if (n > 2) 
          {
              F(0, 2) = 0.5 * dt * dt;
          }
  
          // Define the control matrix B (optional)
          B = Eigen::MatrixXd::Zero(n, 1);
  
          // Define the initial state estimate and covariance matrix
          x = Eigen::MatrixXd::Zero(n, 1);
          P = Eigen::MatrixXd::Identity(n, n);
  
          // Process noise covariance matrix (Q) and measurement noise covariance matrix (R)
          Q = Eigen::MatrixXd::Identity(n, n);
          R = Eigen::MatrixXd::Identity(n, n);
  
          // Define measurement matrix H
          H = Eigen::MatrixXd::Identity(n, n);
      }
  
      Eigen::MatrixXd predict(const Eigen::MatrixXd& u = Eigen::MatrixXd::Zero(1, 1)) 
      {
          x = F * x + B * u;
          P = F * P * F.transpose() + Q;
          return x;
      }
  
      std::pair<Eigen::MatrixXd, Eigen::MatrixXd> update(const Eigen::MatrixXd& z) 
      {
          assert(z.rows() == n);
          Eigen::MatrixXd y = z - H * x;
          Eigen::MatrixXd S = H * P * H.transpose() + R;
          Eigen::MatrixXd K = P * H.transpose() * S.inverse();
  
          x = x + K * y;
          P = (Eigen::MatrixXd::Identity(n, n) - K * H) * P;
  
          return std::make_pair(x, P);
      }
  
  private:
      int n;
      double dt;
      Eigen::MatrixXd F, B, x, P, Q, R, H;
};

int main() 
{
    KalmanFilter kf;

    // Simulate some data
    Eigen::MatrixXd true_state(3, 1);
    true_state << 0, 1, 0.1;
    std::vector<Eigen::MatrixXd> measurements;
    std::vector<Eigen::MatrixXd> predicted_states;

    for (int t = 0; t < 10; t++) 
    {
        // Simulate measurement
        Eigen::MatrixXd measurement = true_state + Eigen::MatrixXd::Random(3, 1);
        measurements.push_back(measurement);

        // Predict step
        kf.predict();

        // Update step
        Eigen::MatrixXd estimated_state;
        Eigen::MatrixXd cov;
        std::tie(estimated_state, cov) = kf.update(measurement);
        predicted_states.push_back(estimated_state);

        // Update true state (for simulation purposes)
        true_state = kf.F * true_state;
    }

    // Plot results
    plt::figure();
    for (int i = 0; i < 3; i++) 
    {
        plt::subplot(3, 1, i + 1);
        std::vector<double> meas_data, pred_data;
        for (const auto& m : measurements) 
        {
            meas_data.push_back(m(i));
        }
        for (const auto& p : predicted_states) 
        {
            pred_data.push_back(p(i));
        }
        plt::plot(meas_data, "r-", "Measurements");
        plt::plot(pred_data, "b-", "Kalman Filter Prediction");
        plt::legend();
    }
    plt::show();

    return 0;
}

