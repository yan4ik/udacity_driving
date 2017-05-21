# PID Controller

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://github.com/udacity/CarND-PID-Control-Project/releases) in the classroom.

Build:
```
mkdir build
cd build
cmake ..
make
```

### Discussion

I chose a manual approach for parameter tuning. My logic was the following:
 * **P** is the main pushing force behind our controller - the bigger the CTE is, the stronger it will push the car towards the center. If it's set to zero, the car just won't turn. So I increased P until the car started turning.
 * When we make P large enough, we observe that our car starts oscillating. To reduce this effect we need to take into account the derivative of CTE - this is what **D** stands for. So I increased D until the car stopped oscillating.
 * Then I observed that the car didn't make it in some abrupt turns. I decided to add a little more pushing force - increase P. But then the car started to oscillate - so I fixed it with increasing D. And basically I repeated it until the car finished the track.
 * One more component - **I** - accumulates all CTE accross the whole track. This helps reduce the systematic bias. However in the simulator I observed a very small bias, so this component was left close to zero.

To keep the steering value between [-1, 1] I used a shifted sigmoid function (tanh could also be used here).

I also decided to use a PID controller for throttle. The idea is simple - if CTE is big, than speed needs to be small.

Here are my final parameters:
```c++
    pid.Init(.6, 0.0001, 12.5);
    th_pid.Init(0.13, 0. , 1.);
```
