#!/usr/local/bin/python3

from random import random, randint
from collections import namedtuple

import matplotlib.image as mpimg
import numpy as np
import cv2
from PIL import Image, ImageEnhance

### ----- Constants ----- ###

# preprocessing constants

HORIZON_LEVEL = 65
MOST_BOTTOM = 125

HORIZONTAL_SHIFT = 30
SHIFT_STEERING_COEF = 0.002
CAMERA_STEERING_SHIFT = 0.2

FLIP_PROB = 0.3
DROP_PROB = 0.1

# training constants

DROPOUT_RATE = 0.2
MINI_BATCH_SIZE = 256

### ----- My Data Pipeline ----- ###

Datapoint = namedtuple('Datapoint',
                       'center_im, left_im, right_im, steering')

data_list = []

# load metadata

with open("data/driving_log.csv") as rf:
    next(rf)

    for line in rf:
        center_im, left_im, right_im, steering, _, _, _ = line.rstrip().split(', ')
        data_list.append(Datapoint(center_im, left_im, right_im, float(steering)))

total_datapoints = len(data_list)

# data generators

def preprocessing(img, steering):
    # crip image
    img = img[HORIZON_LEVEL:MOST_BOTTOM]

    # random shifting
    shift = randint(-HORIZONTAL_SHIFT, HORIZONTAL_SHIFT)
    M = np.float32([[1, 0, shift],[0, 1, 0]])
    img = cv2.warpAffine(img, M, img.shape[1::-1])

    if shift > 0:
        img = img[:,shift:]
    elif shift < 0:
        img = img[:,:shift]

    steering += shift * SHIFT_STEERING_COEF

    # flip image
    if random() < FLIP_PROB:
        img = cv2.flip(img, 1)
        steering *= -1

    # resize
    img = cv2.resize(img, (100, 64), interpolation=cv2.INTER_AREA)

    # randomly change brightness
    img = Image.fromarray(img)
    br = ImageEnhance.Brightness(img)
    img = np.asarray(br.enhance(np.random.uniform(0.8, 1)))

    return img, steering


def single_point_generator():
    while 1:
        shuff_idx = np.random.permutation(total_datapoints)
        for index in shuff_idx:
            yield index


def data_generator():
    g = single_point_generator()

    while 1:

        random_index = next(g)
        datapoint = data_list[random_index]

        center_im = datapoint.center_im
        steering = datapoint.steering
        left_im = datapoint.left_im
        right_im = datapoint.right_im

        if steering == 0:
            if random() < DROP_PROB:
                continue

        camera = randint(1, 3)

        if camera == 1:
            img = mpimg.imread('data/' + center_im)
            eq, steering_res = preprocessing(img, steering)
            yield eq, steering_res
        elif camera == 2:
            img = mpimg.imread('data/' + left_im)
            eq, steering_res = preprocessing(img, steering + CAMERA_STEERING_SHIFT)
            yield eq, steering_res
        elif camera == 3:
            img = mpimg.imread('data/' + right_im)
            eq, steering_res = preprocessing(img, steering - CAMERA_STEERING_SHIFT)
            yield eq, steering_res


def batch_generator():
    g = data_generator()

    while 1:
        batch = [next(g) for _ in range(MINI_BATCH_SIZE)]

        features = np.array([x[0] for x in batch], dtype=np.float32)
        labels = np.array([x[1] for x in batch], dtype=np.float32)

        yield (features, labels)

### ----- My Model ----- ###

from keras.models import Sequential
from keras.layers import Convolution2D, MaxPooling2D
from keras.layers import Dropout, Flatten, Dense, Lambda, ELU
from keras.optimizers import Adam

model = Sequential()

model.add(Lambda(lambda x: x / 255. - 0.5, input_shape=(64, 100, 3)))

model.add(Convolution2D(8, 3, 3, border_mode="same", init="he_normal"))
model.add(ELU())
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Dropout(DROPOUT_RATE))

model.add(Convolution2D(16, 3, 3, border_mode="same", init="he_normal"))
model.add(ELU())
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Dropout(DROPOUT_RATE))

model.add(Convolution2D(32, 3, 3, border_mode="same", init="he_normal"))
model.add(ELU())
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Dropout(DROPOUT_RATE))

model.add(Convolution2D(64, 3, 3, border_mode="same", init="he_normal"))
model.add(ELU())
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Dropout(DROPOUT_RATE))

model.add(Convolution2D(128, 3, 3, border_mode="same", init="he_normal"))
model.add(ELU())
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Dropout(DROPOUT_RATE))

model.add(Flatten())

model.add(Dense(128, init="he_normal"))
model.add(ELU())

model.add(Dense(64, init="he_normal"))
model.add(ELU())

model.add(Dense(16, init="he_normal"))
model.add(ELU())

model.add(Dense(1))

adam = Adam(lr=0.0003)
model.compile(loss='mean_squared_error',
              optimizer=adam,
              metrics=['mean_squared_error'])


train_generator = batch_generator()

model.fit_generator(train_generator,
                    samples_per_epoch=20224,
                    nb_epoch=10,
                    verbose=1)

model.save_weights("model.h5")

with open("model.json", "w") as wf:
    wf.write(model.to_json())
