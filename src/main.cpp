#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include <boost/range/combine.hpp>

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          mpc.start_time = chrono::system_clock::now();

          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];

          // transform waypoints to car coordinates
          // implies car-relative px, py, & psi are 0
          vector<double> carptsx, carptsy;
          for (auto && w: boost::combine(ptsx, ptsy)) {
            double dx = w.get<0>() - px;
            double dy = w.get<1>() - py;
            double carptx = dx * cos(-psi) - dy * sin(-psi);
            double carpty = dx * sin(-psi) + dy * cos(-psi);

            carptsx.push_back(carptx);
            carptsy.push_back(carpty);
          }

          // convert to Eigen vectors for polyfit
          Eigen::VectorXd wayptsx = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(carptsx.data(), carptsx.size());
          Eigen::VectorXd wayptsy = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(carptsy.data(), carptsy.size());

          // fit a polynomial to the above x and y coordinates
          // default order = 3, thus ...
          // f(x) = coeffs[0] + coeffs[1] * x + coeffs[2] * x^2
          auto coeffs = polyfit(wayptsx, wayptsy, POLYORDER) ;

          // convert v (mph) to m/s
          v *= 0.44704;

          // calculate the cross track error
          // cte = y - f(x)
          // double cte = py - polyeval(coeffs, px);
          double cte = polyeval(coeffs, 0);
          // calculate the orientation error
          // epsi = psi - arctan(f'(x))
          // regardless of POLYORDER, f'(x) = coeffs[1] since px = 0
          double epsi = -atan(coeffs[1]);

          /*
          * Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          // x, y, & psi are all 0 in vehicle coordinates - let's be explicit
          px = py = psi = 0;

          // Take latency into account, all calculations in car coordinates
          // Using actual time spent in message call (including LATENCY from MPC.h)
          // Here are the motion equations in world space:
          //  x = x + v * cos(psi) * dt
          //  y = y + v * sin(psi) * dt
          //  psi = psi + v / Lf * delta * dt
          //  v = v + a * dt, use throttle ~ a
          //  cte = y - f(x) + v * sin(epsi) * dt
          //  epsi = psi - arctan(f'(x)) + v * delta / Lf * dt
          px = v * mpc.elapsed_time.count();   // x is just v*t
          py = 0;             // y is the same
          psi = v * -steer_value * mpc.elapsed_time.count()/Lf;
          v += throttle_value * mpc.elapsed_time.count();
          cte += v * sin(epsi) * mpc.elapsed_time.count();
          epsi += v * -steer_value * mpc.elapsed_time.count()/Lf;

          Eigen::VectorXd state(6);
          state << px, py, psi, v, cte, epsi;
          mpc.Solve(state, coeffs);

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          steer_value = mpc.steer_value;
          throttle_value = mpc.throttle_value;
          msgJson["steering_angle"] = -steer_value / 0.436332;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc.x_values;
          msgJson["mpc_y"] = mpc.y_values;

          //Display the waypoints/reference line
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          msgJson["next_x"] = carptsx;
          msgJson["next_y"] = carptsy;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(int(LATENCY*1000)));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          mpc.end_time = chrono::system_clock::now();
          mpc.elapsed_time = mpc.end_time - mpc.start_time;
          //cout << "Elapsed Time: " << mpc.elapsed_time.count() << "s\n";
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
