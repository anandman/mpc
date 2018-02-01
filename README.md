# Model Predictive Controller

This is a sample implementation of a Model Predictive Controller (MPC) to maneuver a vehicle around a track.

---

## MPC Background

## MPC Implementation

## MPC Tuning

## Results

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
