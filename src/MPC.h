#ifndef MPC_H
#define MPC_H

#include <vector>
#include <chrono>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

// order of polynomial to fit waypoints to
#define POLYORDER 3
// latency of simulator commands in seconds
#define LATENCY 0.1

// Set the timestep length and duration
const size_t N = 10;
const double dt = 0.15;

// reference velocity to start with
// 1 mph -> 0.44704 m/s
const double ref_v = 90 * 0.44704;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// cost coefficients to weight different things
const double cteCoeff = 2;          // cost of cross-track error
const double epsiCoeff = 1;         // cost of psi error
const double vCoeff = 0.1;          // cost of speed changes
const double deltaCoeff = 5000;     // cost of steering angle change
const double aCoeff = 1;            // cost of throttle change
const double seqDeltaCoeff = 400;   // cost of steering angle changes over time
const double seqACoeff = 200;       // cost of throttle change over time

using namespace std;

double polyeval(Eigen::VectorXd coeffs, double x);
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

class MPC {
  public:
    MPC();

    virtual ~MPC();

    chrono::time_point<chrono::system_clock> start_time;
    chrono::time_point<chrono::system_clock> end_time;
    chrono::duration<double> elapsed_time;
    double steer_value;
    double throttle_value;
    std::vector<double> x_values;
    std::vector<double> y_values;

    // Solve the model given an initial state and polynomial coefficients.
    void Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
