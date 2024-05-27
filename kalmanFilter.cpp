
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
          F = MatrixXd::Identity(n, n);
          for (int i = 1; i < n; i++) 
          {
              F(i - 1, i) = dt;
          }
          if (n > 2) 
          {
              F(0, 2) = 0.5 * dt * dt;
          }
  
          // Define the control matrix B (optional)
          B = MatrixXd::Zero(n, 1);
  
          // Define the initial state estimate and covariance matrix
          x = VectorXd::Zero(n);
          P = MatrixXd::Identity(n, n);
  
          // Process noise covariance matrix (Q) and measurement noise covariance matrix (R)
          Q = MatrixXd::Identity(n, n);
          R = MatrixXd::Identity(n, n);
  
          // Define measurement matrix H
          H = MatrixXd::Identity(n, n);
      }
  
      VectorXd predict(const VectorXd& u = VectorXd::Zero(1)) 
      {
          x = F * x + B * u;
          P = F * P * F.transpose() + Q;
          return x;
      }
  
      std::pair<VectorXd, MatrixXd> update(const VectorXd& z) 
      {
          assert(z.size() == n);
          VectorXd y = z - H * x;
          MatrixXd S = H * P * H.transpose() + R;
          MatrixXd K = P * H.transpose() * S.inverse();
  
          x = x + K * y;
          P = (MatrixXd::Identity(n, n) - K * H) * P;
  
          return std::make_pair(x, P);
      }
  
  private:
      int n;
      double dt;
      MatrixXd F, B, P, Q, R, H;
      VectorXd x;
};

int main() 
{
    KalmanFilter kf;

    // Simulate some data
    VectorXd true_state(3);
    true_state << 0, 1, 0.1;
    std::vector<VectorXd> measurements;
    std::vector<VectorXd> predicted_states;

    for (int t = 0; t < 10; t++) 
    {
        // Simulate measurement
        VectorXd measurement = true_state + VectorXd::Random(3);
        measurements.push_back(measurement);

        // Predict step
        kf.predict();

        // Update step
        VectorXd estimated_state;
        MatrixXd estimated_covariance;
        std::tie(estimated_state, estimated_covariance) = kf.update(measurement);
        predicted_states.push_back(estimated_state);

        // Update true state (for simulation purposes)
        true_state = kf.F * true_state;
    }

    // Plot results
    plt::figure();
    for (int i = 0; i < 3; i++) 
    {
        plt::subplot(3, 1, i + 1);
        std::vector<double> measurement_values, prediction_values;
        for (const auto& m : measurements) 
        {
            measurement_values.push_back(m(i));
        }
        for (const auto& p : predicted_states) 
        {
            prediction_values.push_back(p(i));
        }
        plt::plot(measurement_values, "r-", {{"label", "Measurements"}});
        plt::plot(prediction_values, "b-", {{"label", "Kalman Filter Prediction"}});
        plt::legend();
    }
    plt::show();

    return 0;
}

