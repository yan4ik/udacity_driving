# Sensor Fusion using the Unscented Kalman Filter

[//]: # (Image References)

[image1]: ./images/1.fusion.png
[image2]: ./images/2.fusion.png
[image3]: ./images/laser_nis.png
[image4]: ./images/radar_nis.png


Project structure:
 * src - directory with source files
 * data - directory with test data
 * output - directory with my output results in the format {dataset_num}.{sensor_type}.txt

Build:
```
mkdir build
cd build
cmake ..
make
```

Usage:
```
Usage instructions: ./UnscentedKF path/to/input.txt path/to/output.txt
```

### Dataset 1 Results

#### Fusion (both sensors)

![alt text][image1]

rmse:
```
0.05204
0.04444
0.56973
0.53178
```

### Dataset 2 Results

#### Fusion (both sensors)

![alt text][image2]

rmse:
```
0.160818
0.175743
0.253297
0.321405
```

### Discussion

We can see that the Unscented Kalman Filter did a better job than the Extended Kalman Filter. This can be clearly seen on Dataset 1. Extended Kalman Filter failed to track the object on turns - it significantly overshot the ground truth trajectory. But here everything is great - thanks to the nonlinear CTRV process model.

The implementation was pretty straightforward, with the exception that on Dataset 2 I experianced numerical instability issues. But with careful parameter tuning and chunking big time intervals (thanks to udacity forums!) this was solved.

My model passes needed benchmarks but still can be improved. If we look as NIS graphs for laser and radar on Dataset 2 we can see that I underestimate laser noise and overestimate radar noise.

![alt text][image3]

![alt text][image4]
