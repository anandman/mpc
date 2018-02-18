# Model Predictive Controller

This is a sample implementation of a Model Predictive Controller (MPC) to maneuver a vehicle around a track.

---

## MPC Background
Model Predictive Control (MPC) is a smoother and ultimately, more realistic, control algorithm than a PID controller. It is also more complex as it uses actual motion control equations to model a path over a time horizon to follow a reference path. Imagine if you will a human driver trying to follow a ghost path in a video game. Here are the motion control equations used. We use a bicycle model which assumes two wheels with a turning radius of *Lf*. 

<p align="center">
<img src="images/motion_control_eqs.svg">
</p>

## MPC Implementation
This implementation of an MPC is in C++ using the Ipopt optimzer and CppAD automatic differentiation libraries to set the steering angle and throttle controls while driving a simulated vehicle around a race track following reference waypoints. In a real implementation, the reference waypoints might be derived using path planning algorithms.

At each timestep, *t*, the vehicle's state (global position, heading, speed, steering angle, and throttle) and the next six waypoints are received by the controller. These waypoints are plotted as a yellow line for reference.

First, we convert the waypoints from global to vehicle coordinate systems. We then fit a 3rd order polynomial to the waypoints for use as the reference driving path. We derive the cross-track error (CTE) & heading error (Eψ) from this polynomial. The final step before applying the MPC is to derive the current state of the vehicle after the latency in the simulator and MPC algorithms using the motion control equations.

We then use the polynomial and estimated current state of the car to run the MPC to derive an optimized driving path over a 1.5s horizon (*N*=10 * *dt*=0.15s = 1.5s). The MPC uses a cost optimization function (see below) to find optimal solutions over the 1.5s for the model control equations. These first of these solutions are then used to steer and throttle the vehicle and the rest plotted as a green line.

<p align="center">
<img src="images/cost_function.svg">
</p>

## MPC Tuning
There are only a handful of coefficients to tune. The first is the timestep length (*N*) and duration (*dt*). N was chosen as a small value and dt to ensure the total time horizon was a little over 1s, in this case, 1.5s.

The second set of parameters to tune are the cost coefficients (_w*_) from the cost function equation above. These were tuned by hand for this course, ensuring that any change to the steering angle, whether step-to-step or over time, incurs the highest cost. Note that a different course would require tuning again. Obviously, this is very inefficient and we would need to find a way to autotune these parameters in the future.  

## Results
Here is the result of running the MPC on the sample track in the simulator. Click on the preview to see the full video.
<p align="center">
<a href="images/MPC_track.mp4"><img src="images/MPC_track.gif"></a>
</p>

## Dependencies

* [Udacity Self-Driving Car Simulator](https://github.com/udacity/self-driving-car-sim/releases)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Linux: run the [`install-ubuntu.sh`](install-ubuntu.sh) script in the repository
  * Mac: run the [`install-mac.sh`](install-mac.sh) script in the repository
  * Windows: use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10)
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x.
* **Ipopt and CppAD:** Please refer to [this document](install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* [Boost](http://www.boost.org)
  * Linux: See [these directions](http://www.boost.org/doc/libs/1_66_0/more/getting_started/unix-variants.html)
  * Mac: `brew install boost` or same directions as Linux
  * Windows: See [these directions](http://www.boost.org/doc/libs/1_66_0/more/getting_started/windows.html)
* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Build and Run Instructions

1. Clone this repo
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

1. Clone this repo
2. Install uWebSocketIO as indicated [above](#dependencies)
3. Make a build directory: `mkdir build && cd build`
4. Compile: `cmake .. && make`
5. Run it: `./mpc`
6. Start Udacity simulator application
    1. Select resolution, graphics quality, etc., and press "Play!"
    2. Select "Project 5: MPC Controller"
