# PID Controller

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
 * One more component - **I** - accumulates all CTE accross the whole track. This helps reduce the systematic bias. However in the simulator I observed a very small bias, so I was left close to zero.

I also decided to use a PID controller for throttle. The idea is simple - if CTE is big, than speed needs to be small.

Here are my final parameters:
```c++
    pid.Init(.6, 0.0001, 12.5);
    th_pid.Init(0.13, 0. , 1.);
```
