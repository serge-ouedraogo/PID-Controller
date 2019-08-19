# PID Controller Code Documentation
Self-Driving Car Engineer Nanodegree Program

---
## The Proportional Component: P
The proportional component P is a gain directly proportional to the cross-track error (cte). An increase of this gain leads to a strady oscillation of the vehicle.

## The Integral Component: I
This component is a gain that reflects the integral of the cte over time. For this code, this component is calculated every 600 steps, after the vehicle has accumulated errors over time. The visual effect is a reduction of the oscillations observed when P only was turned on. 

## The Derivative Component: D
The D component is a gain that is proportional to the rate of change of the cte. A perfectly tuned D component will decrease the amplitude of the oscillations (damped oscillation)


# Optimization
 The twiddle algorithm was used to choose the hyperparameters. Twiddle continuously tunes the PID controller hyperparameters (P, I, D) by first calculating the cte and incrementally ajusts the coefficients until the cte is minimized. 
 
## Dependencies

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 


