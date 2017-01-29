#Advanced Lane Finding project

[//]: # (Image References)

[image1]: ./images/undistort.example.png "Undistorted"

Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

All the code is located in "adv_lane_detect.ipynb".

---

###Camera calibrtaion

The code is located in cells 5-6. It's an exact copy of a tutorial given in the lectures - `cv2.findChessboardCorners`, `cv2.calibrateCamera` and `cv2.undistort` does all the job for us.

Here's one example of what I got:
![alt text][image1]

More examples can be found in cell 7 in the notebook. There I tested my distortion correction on images where opencv failed to detect corners for some reason.
