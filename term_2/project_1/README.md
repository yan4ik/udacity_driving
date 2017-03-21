# Sensor Fusion using the Extended Kalman Filter

[//]: # (Image References)

[image1]: ./images/1.radar.png

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
