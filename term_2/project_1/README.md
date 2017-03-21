# Sensor Fusion using the Extended Kalman Filter

[//]: # (Image References)

[image1]: ./images/1.radar.png
[image2]: ./images/1.radar.png
[image3]: ./images/1.fusion.png

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

Radar

![alt text][image1]

rmse:
```
0.10121
0.0823314
0.613423
0.581877
```

Laser

![alt text][image2]

rmse:
```
0.0681865
0.0572297
0.625587
0.560902
```

Fusion (both sensors)

![alt text][image3]

rmse:
```
0.0651649
0.0605378
0.54319
0.544191
```
