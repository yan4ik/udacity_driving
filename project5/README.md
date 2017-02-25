**Vehicle Detection Project**

[//]: # (Image References)
[image1]: ./examples/128x128.jpg
[image2]: ./examples/75x75.jpg
[image3]: ./examples/overlap_hell.jpg
[image4]: ./examples/result.jpg

---

First and most important: [my video result](https://www.youtube.com/watch?v=WbT41thK7dg)

### Data

Besides the data provided, I used the first dataset from [here](https://github.com/udacity/self-driving-car/tree/master/annotations). My code for preparing this dataset can be found in the notebook under section "Annotated Driving Dataset".

The process goes as follows:
 - for vehicle images I obviously use the annotated frames (reshaped to 64x64).
 - for non-vehicle images I randomly select a crop of variable size from an image. A little trick here is that I select such frames, that do not overlap with vehicle frames.

I ended up with over 100 000 images.

### Train / Validation split

It is worth noticing that if you apply a common random split into train / validation datasets, you will end up with 99% accuracy on both sets but a terrible performance on test images. This is clear overfitting. The source of it is the nature of how the data was collected. Since we are dealing with videos we have a lot of frames which a close to each other in time, and hence very similar. So a random split will shuffle these frames between train and validation datasets, and as a result they will become very similar. This is clearly bad, since we loose a clear metric that will help us tune our algorithm. In order to solve this problem, I used the fact that we have multiple different sources of video data, and the right way to go is to split on those sources. So I decided to use the `vehicles/KITTI_extracted/` and `non-vehicles/Extras/` folders only for validation.

### Features

I ended up only using the HOG features. Here is my setup:
```python
def get_features(image):
    gray = skimage.color.rgb2gray(image)
    hog_features = hog(gray, 
                       orientations=10,
                       pixels_per_cell=(12, 12),
                       cells_per_block=(1, 1),
                       transform_sqrt=True)

        
    return hog_features
```

I do not have a clear path towards this particular combination, it was mostly just a matter of experimentaion and trying different combinations. Though I made two decisions consciously:
 * I decided to drop all color information, since cars can be of any color. All that matters is the shape.
 * I tried to make the dimensionality of the resultng feature vector not too large, in order to combat overfitting.

As an additional preprocessing step a used simple feature normalizetion:
```
# Preprocessing: Subtract the mean feature
mean_feat = np.mean(X_train, axis=0, keepdims=True)
X_train -= mean_feat
X_val -= mean_feat

# Preprocessing: Divide by standard deviation.
std_feat = np.std(X_train, axis=0, keepdims=True)
X_train /= std_feat
X_val /= std_feat
```

### The Model

I experimented with a lot of models: Linear SVM, Kernel SVM, Random Forest, Gradient Bossting, ...

I ended up with a pytorch single hidden layer neural network, because it -is cool- gave me the highest validation accuracy (91%). Also, given current tools, neural nets are the most flexible and convenient nonlinear model you can train on a large dataset.

My architecture is the following: 
```python
class MyAwesomeNet(nn.Module):


    def __init__(self):
        super(MyAwesomeNet, self).__init__()

        self.fc1 = nn.Linear(250, 50)
        self.fc1_bn = nn.BatchNorm1d(50)
                
        self.fc_final = nn.Linear(50, 2)


    def forward(self, x):
        x = F.elu(self.fc1_bn(self.fc1(x)))
        x = self.fc_final(x)

        return x
```

### Boxing

In order to choose the configuration for candidate windows I followed one principle - in order to combat false positives we need to have multiple overlapping windows covering the same car. This will help us get rid of random classifier decisions.

I ended up using three types of windows:
 * 128x128 - to search for big cars
 * 75x75 - to serach for small cars near the horizon level
 * 96x96 - to give additional "overlappiness"

The most difficult thing was to get solid performance near the horizon level. 75x75 frames are so small that it is hard for them to trigger on the same car. That is why I gave them a pretty big overlap in the x direction - 0.9 (as an afterthought maybe this is to extreme and can be tuned better in order to save computation time).

Here are some examples of my individual windows in action:

![alt text][image1]
![alt text][image2]

Here is an example of a full-blown collection of all triggered windows:

![alt text][image3]

As was suggested in udacity tutorials, we can use a heatmap to nail good boxes. I use a combination of two heatmaps:
 * in each frame I drop pixels that got less than two triggered windows.
 * I also maintain heatmaps for the last 10 frames, and the final boxes are drawn on pixels who were present in at least 7 frames.

As an additional hack, I "reset" active pixels on heatmaps to 1 (`heatmap[heatmap != 0] = 1`). This helps me get more stable boxes.

My final boxes on test images can be found in `examples` directory. Here is one such image:
![alt text][image4]

# Discussion

The final result is satisfactory but far from something deployable:
 * the boxes are wiggly, and are not tight enough.
 * I search only in the right-lower piece of the whole frame, so if a car will turn right in front of me, I will miss it.
 * when two cars come close together, their frame merges into one.
 * the whole pipeline is not fast enough.

It will be insteresting to experiment further with this problem by using modern deep learning approaches (u-net, YOLO etc.).
