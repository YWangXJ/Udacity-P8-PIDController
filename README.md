# CarND-Controls-PID

Self-Driving Car Engineer Nanodegree Program

## Objective
The objective of this project is to implement the PID controller for the steering of the vehicle and use Twiddle to find a safe parameters for PID. (without runing off road)

## Reflection
* The "P" is the proportional part and generate valuse proportional to Cross Track Error (CTE). The CTE is the error between the center of the road and the vehicle's position as the cross track error (CTE). This makes the car tends to stay at the center of the road by taking large steering angle when the car is far off the center.
* The "I" is the integral sum of CTE, which prevent the car from staying only at one side of the road.
* The "D" is the derivate records the change of CTE. On the one hand, it will produce large change to the steering to let the car change back to center more quickly when the car is far off the center. On the other hand, it will produce minor change ot the steering to let car smoothly return to the center.
To let the car run safely on the road, Twiddle is used to find the best parameters (safe, but maybe not efficient of comfortable)
* The Twiddle method is initialized with P, I, D = {0.3, 0.01, 3} and finalized at {1.37989,0.0464046,15.3226}.
* It was found that the initial value of parameters is important in this project. If it is not correctly selected, the car may run off the road at the begining. Thus several rounds of tests had been conducted to find a good inital set of values
* According to the results from twiddle, the P, I, D are set to {1.37989,0.0464046,15.3226}.

With the selected parameters, the car is able to run safely without runing of the road for a lap. However, due to the large D selected, the steering oscillates a lot and won't be comfortable enough for drivers. Futher adujustment could be done to improve that.



---

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

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)



