# MPC Control
Udacity Self-Driving Car Nanodegree Term 2, Project 5

## Project Basics
Using Model Predictive Control (MPC), this project involves writing a C++ program that can drive a simulated car around a virtual track using specific waypoints from the track itself. The simulated car's actuators have a 100ms latency (delay) that must be accounted for as well as part of the MPC calculation.

### Project Steps
* Fitting a line based on road waypoints and evaluating the current state based on that polynomial line.
* Implementing the MPC calculation, including setting variables and constraints
* Calculating actuator values from the MPC calc based on current state
* Accounting for latency (I used a predicted state 100ms in the future to replace the actual current state in the calculation)
* Calculating steering angle & throttle/brake based on the actuator values
* Setting timestep length and duration
* Testing/tuning of above implementations on Udacity simulator

### Results
See video of the results from my implementation [here](MPC_vid.mov). The car is able to approach speeds of nearly 100 mph, with only a few hitches in its planned route (which it self-corrects for in the following timestep).

## Discussion/Reflection
### The Model
My MPC model starts out by taking in certain information from the simulator: 
* ptsx (x-position of waypoints ahead on the track in global coordinates)
* ptsy (y-position of waypoints ahead on the track in global coordinates)
* px (current x-position of the vehicle's position in global coordinates)
* py (current y-position of the vehicle's position in global coordinates)
* psi (current orientation angle of the vehicle, converted from the simulator's format to that expected in mathematical formulas)
* v (current velocity of the vehicle)
* delta (current steering angle of the car, i.e. where the wheels are turned, as opposed to the actual orientation of the car in the simulator at that point [psi])
* a (current throttle)

#### Polynomial Fitting & Preprocessing
Now, in order to simplify the calculations, I transform the points from the simulator's global coordinates into the vehicle's coordinates. This is done in lines 102-107 of `main.cpp`. First, each of the waypoints are adjusted by subtracting out px and py accordingly such that they are based on the vehicle's position. Next, the waypoint coordinates are changed using standard 2d vector transformation equations to be in vehicle coordinates:
* ptsx_car[i] = x * cos(-psi) - y * sin(-psi)
* ptsy_car[i] = x * sin(-psi) + y * cos(-psi)

Using the `polyfit()` function, a third-degree polynomial line is fit to these transformed waypoints, essentially drawing the path the vehicle should try to travel. Moving on further is where the transformations are critical - because we are operating from the vehicle's coordinates, we can use px, py and psi all equal to zero: from the vehicle's standpoint, it is the center of the coordinate system, and it is always pointing to a zero orientation. The cross-track error can then be calculated by evaluating the polynomial function (`polyeval()`) at px (which in this case is now zero, so technically could also just be calculated as the first coefficient value - i.e. the one with a zero-order x). The psi error, or epsi, which is calculated from the derivative of polynomial fit line, is therefore simpler to calculate, as polynomials above the first order in the original equation are all eliminated through multiplication by zero (since x is zero). It is the negative arc tangent of the second coefficient (the first-order x was in the original polynomial).

#### Accounting for Latency
In what was perhaps the most important aspect of this project, my model then accounts for the simulator's added 100ms latency between the actuator calculation (when the model tells the car to perform a steering or acceleration/braking change) and when the simulator will actually perform that action. I originally tried to account for this by changing the N and dt values within `MPC.cpp`, but found that to be in an incorrect approach, as while my initial attempts held the line well at the beginning, it always failed to initiate a turn in time to not run off the track at the first curve.

To implement this, I added in a step to predict where the vehicle would be after 100ms (0.1 seconds), in order to take the action that needed to actually be taken at that time, instead of the one in reaction to an old situation. I set the "dt" value here (not to be confused with the one in `MPC.cpp`, although both are the same value) to equal the latency. Then, using the same update equations as those used in the actual MPC model, I predicted the state and fed that into the true model. Note that these equations were able to be simplified again because of the coordinate system transformation - using x, y and psi all of zero made these equations a little simpler, as lots of the values end up being zero or one. See lines 131-143 in `main.cpp`. This new predicted state, along with the coefficients, are then fed into the `mpc.Solve()` function found in `MPC.cpp`.

#### MPC.cpp - Where the Magic Happens
**Coming soon**
* To discuss overall variables and constraints
* To discuss weights for cost functions

### Back to The Simulator
**Coming soon**
* To discuss what gets returned from MPC.cpp and how to draw the lines

#### Tuning Timesteps (N) and Timestep Duration (dt) in MPC.cpp
**Coming soon**

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
