#Advanced Lane Finding project

[//]: # (Image References)

[image1]: ./images/undistort.example.png "Undistorted"
[image2]: ./images/transformed.example.png "Transformed"
[image3]: ./images/thresholded.example.png "Thresholded"

Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

All the code is located in "adv_lane_detect.ipynb".

---

###Camera calibrtaion

The code is located in cells 5-6. It's an exact copy of a tutorial given in the lectures - `cv2.findChessboardCorners`, `cv2.calibrateCamera` and `cv2.undistort` does all the job for us.

Here's one example of what I got:

![alt text][image1]

More examples can be found in cell 7 in the notebook. There I tested my distortion correction on images where opencv failed to detect corners for some reason. In cell 8 you can find my distortion correction applied to test images.

###Perspective Transform

The code is located in cell 9. Again, nothing very impressive on my side - `cv2.getPerspectiveTransform` and `cv2.warpPerspective` are my friends here. A little nontrivial was to carefully adjust the coordinates of source and destination points. But after some trial and error I managed to get it right.

Here's one example of my perspective transform:

![alt text][image2]

More examples can be found in cell 10.

###Thresholding

The code is located in cell 11. I use simple color masks - one white and one yellow, both applied to a BGR image. White mask is trivial - just take pixels where all channels are high enough. Yellow mask is more tricky to implement, but I ended up with a pretty robust rule - I select pixels with high enough red channel and high enough difference between green and blue channels.

This simple strategy gives pretty good results, slightly better than what I tried to achieve with gradient thresholding. In my case gradients gave a lot of noise, so I decided to stick with color masks.

Here's one example:

![alt text][image3]

More examples can be found in cell 12.

