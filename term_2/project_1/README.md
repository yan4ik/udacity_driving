# Sensor Fusion using the Extended Kalman Filter

[//]: # (Image References)

[image1]: ./images/1.radar.png
[image2]: ./images/1.laser.png
[image3]: ./images/1.fusion.png
[image4]: ./images/2.radar.png
[image5]: ./images/2.laser.png
[image6]: ./images/2.fusion.png


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
Usage instructions: ./ExtendedKF path/to/input.txt output.txt
```

### Dataset 1 Results

#### Radar

![alt text][image1]

rmse:
```
0.10121
0.0823314
0.613423
0.581877
```

#### Laser

![alt text][image2]

rmse:
```
0.0681865
0.0572297
0.625587
0.560902
```

#### Fusion (both sensors)

![alt text][image3]

rmse:
```
0.0651649
0.0605378
0.54319
0.544191
```

### Dataset 2 Results

#### Radar

![alt text][image4]

rmse:
```
0.151979
0.204512
0.104861
0.129556
```

#### Laser

![alt text][image5]

rmse:
```
0.217996
0.194325
0.937449
0.833882
```

#### Fusion (both sensors)

![alt text][image6]

rmse:
```
0.186487
0.19027
0.477586
0.807415
```

### Discussion

We can see that in Dataset 1 both sensors are pretty much on par, and the fused result is the best one.

But on Dataset 2 we observe that the best result is achieved by a radar sensor alone! The reason may be that the second dataset has much longer nonlinear segments, and hence our laser sensor fails to capture is properly due to its linearity assumptions.
