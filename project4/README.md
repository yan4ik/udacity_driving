#Advanced Lane Finding project

[//]: # (Image References)

[image1]: ./images/undistort.example.png "Undistorted"
[image2]: ./images/transformed.example.png "Transformed"
[image3]: ./images/thresholded.example.png "Thresholded"
[image4]: ./images/fitted.example.png "Fitted"
[image5]: ./images/final.example.png "Final"

Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

All the code is located in "adv_lane_detect.ipynb".

---

###Camera calibrtaion

The code is located in cells 5-6. It's an exact copy of a tutorial given in the lectures. We are given several photos of a chessboard, where we can find the corners using opencv. And we also know how an undistorted chessboard look like, and where the corners must be there. Using this information we can create a mapping between the distorted points and undistorted points, and this will help us calibrate our camera. `cv2.findChessboardCorners`, `cv2.calibrateCamera` and `cv2.undistort` does all the dirty job for us.

Here's one example of what I got:

![alt text][image1]

More examples can be found in cell 7 in the notebook. There I tested my distortion correction on images where opencv failed to detect corners for some reason. In cell 8 you can find my distortion correction applied to test images.

###Perspective Transform

The code is located in cell 9. The basic idea is to choose "source" points on the original image, and their future "destination" coordinates in the transformed image. Again, nothing very impressive on my side - `cv2.getPerspectiveTransform` and `cv2.warpPerspective` are my friends here. A little nontrivial was to carefully adjust the coordinates of source and destination points. But after some trial and error I managed to get it right.

Here's one example of my perspective transform:

![alt text][image2]

More examples can be found in cell 10.

###Thresholding

The code is located in cell 11. I use simple color masks - one white and one yellow, both applied to a BGR image. White mask is trivial - just take pixels where all channels are high enough. Yellow mask is more tricky to implement, but I ended up with a pretty robust rule - I select pixels with high enough red channel and high enough difference between green and blue channels.

This simple strategy gives pretty good results, slightly better than what I tried to achieve with gradient thresholding. In my case gradients gave a lot of noise, so I decided to stick with color masks.

Here's one example of my approach:

![alt text][image3]

More examples can be found in cell 12.

###Curve fitting

The code is located in cell 13. First, I split the image in two halves, and work with them independently. I use `np.polyfit` to fit a quadratic function.

Here's one example of what I got:

![alt text][image4]

More examples can be found in cell 14.

###Final Result

My final result image looks something like this:

![alt text][image5]

Highlighting is done exactly as suggested by Udacity tutorial - we construct a polygon from our fitted curves and project it back to the original image. The code is in cell 15. More example on test images can be found in cell 16.

###Radius of curvature and position of vehicle

Radius of curvature is calculated by the formula given in the tutorials. Code is in cell 19. On video it oscillates pretty wildly, but on average is pretty close to 1km, which seems reasonable.

The position of vehicle is calculated as follows - we look at the most bottom of our highlighted region and search for the leftmost and rightmost points. We assume that if car is exactly in the center of the road then these points have identical offsets from the borders of the image. So we measure the shift of these points from the center - right is positive, left is negative. We also don't forget to calibrate our measurements from pixel space to real world space. The code is in cell 15. On the video my method gives reasonable results - when the car drives closer to the left yellow line, the shift decreases.

---

And finally here's the [link to my video](https://www.youtube.com/watch?v=QMxZJ0u3C00). it also can be found in `output_videos/project_video_done.mp4`.
I used simple averaging over the last 10 frames. When I can't find lines on the frame, I simply ignore it and use lines from the previous frame.

---

# Reflections

* my approach is really simple, very basic techniques and very little tweaking
* despite simplicity the processing code runs reeeeaaaaaally slooooooow, I can't imagine how this will work on a real car in real-time. Well, I blame it on python =)
* my approach is not robust yet - right now it fails on both challenge videos

What further directions I find most promising:
* a principled way of dealing with brightness - shadows and bright sun are totally killing my approach right now. Simple color masking totally fails here.
* during the video lane lines should be updated using bayesian inference - previous frame gives us prior distribution, current frame gives us likelihood and we need to figure out the posterior. In this framework the whole process will be much more robust.
* the thing that puzzles me the most is how to deal with situation which occures near 00:40 on the harder challenge video. In this episode the right lane totally dissapears, but the "right lane" from the opposite lane clearly comes into picture. This situation are really dangerous - the car can switch to opposite lane, if we're not carefull about it.
