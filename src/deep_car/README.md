# Pixels to Drive

This is the repository for a software project. We trained a neural network to drive the car.
You can downlaod the weights and let the neural network take over.
The network drives the car successfully, i.e. the car circles the track.
However, neural network does not a very good job at avoiding obstacles.

# Dependencies

See the `requirements.txt` file. The packages can be install with `$ pip install -r
requirements.txt`.

# Notebooks

Most code is written in Jupyter Notebooks. Here is a short overview. A more
detailed description can be found inside the notebooks.

1. [rosbag_to_hdf5.ipynb](notebooks/rosbag_to_hdf5.ipynb): Converts the rosbag
   files to a train and test hdf5 file
1. [train.ipynb](notebooks/train.ipynb): Train the neural network.
1. [driver.ipynb](notebooks/driver.ipynb): Connects to the car, loads, and runs the
   neural network live.

# Code

* [`data.py`](deep_car/data.py): Contains augmentation functions and other
  helper functions
* [`model.py`](deep_car/model.py): The neural network architecture is defined
  here.

# Train data collection

For collecting the data we build a little test race circuit. For the first
generation of data we were simply driving the trip manually controlling
the car with the android app. The driving time was about 1 hour. We sticked
to driving in the same direction. In the end we had 3 big files with trainig
data.

For the second generation we placed various obstacles on the driving road.
We used orange soccer balls and some big chess statues. This time we were also
driving in both directions. The driving time was about 1 1/2 hour. Again the
driving was done manually with the android app. This time we split our trainig
data into many smaller data sets.

Finally we combined the data to create our test and train set.

We used the following command for recording the data:

```bash
$ rosbag record  \
    /manual_control/speed \
    /manual_control/steering \
    /model_car/yaw \
    /deepcar/resize_img80x60/compressed
```

The data can be downloaded from here: https://drive.google.com/open?id=0B4-Jw9T9VL8nYTVTbmRfZHVrVDA

Check the sha1sum:

```
527d3561561deae40300da706bc0467a5175719c  rosbags.tar.gz
```

## Network architecture

The input of the neural network is of shape `(48, 64)`.
Given a single frame, the network predicts the steering.

The neural network is made from multiple convolutional blocks.
A convolutional block looks like this:
1. [Convolutional layer](https://en.wikipedia.org/wiki/Convolutional_neural_network) with filter size `5x5`. The feature size vary by depth. The edges are padded such that the output has the
same shape as the input (type same).
1. [Batch Normalization](https://arxiv.org/abs/1502.03167)
1. [Relu](https://en.wikipedia.org/wiki/Rectifier_(neural_networks)) activation.
1. Max-Pooling layer scales the feature map down by a factor of two.

This block is repeat 5 times. The first block has a feature size of 32.
For every block, we double the feature size, e.g. 32, 64, 128, and so on.

After the last convolutional block, we use an averaging layer and then a dense
layer that projects the features to a single scalar which represents the
steeering.

We use mean squared error as loss and train the network with the
[Adam](https://arxiv.org/abs/1412.6980) optimizer.

## Scripts on the car

All notebooks can be run remotely. It is preferrable to scale the camera images on the car to save bandwidth.
Start the appropiate script on the car:

* [`crop_img.py`](scripts/crop_img.py): This crops the image to (64, 40) and converts them to grayscale such that they can be used as input to the neural network. The topic name is `/deepcar/crop_img64x48/compressed`.
*  [`resize_img80x60.py`](scripts/resize_img80x60.py): Crops the image to (80, 60) and leaves them in RGB color space. Run this script if you want to record data. The topic name is `/deepcar/resize_img80x60/compressed`.

## Run the network on the car

You have to install tensorflow on the car. We used the car 103 which has
tensorflow installed:

1. Install the repository by running `pip install -e .` inside this directory.
1. Download the network weights from [here](https://drive.google.com/open?id=0B4-Jw9T9VL8nY054dG45QmRCUHc) into the `data` directory. The sha1 sum should be: `7ae7c7aa2db79cd6c0df149cd3257868dc8e99a7  steering_model.tar.gz`
1. Extract the weights:
     `$ tar -xvf steering_model.tar.gz`
   The weights should now be located at `data/steering_model` where the path is relative to the
   git repository.
1. Run the network:
    ```
    $ cd scripts
    $ python2 driver.py
    ```
1. Release the handbreak and give gas:
    ```
   $ rostopic pub -r 1 manual_control/stop_start std_msgs/Int16  '{data: 1}'   # press control+c after 2 seconds
   $ rostopic pub -r 1 manual_control/speed std_msgs/Int32  '{data: -200}'     # press control+c after 2 seconds
    ```

## Future work

There exists multiple possible directions for future work:
* **Reinformcement learning**: Give an reward for good driving maneuver. A [very simple driving enviroment](https://gym.openai.com/envs/CarRacing-v0)
  exists in OpenAI's gym.
* **Network compression**: Improve the performance by using neural network
  compression, e.g. see this [paper](https://arxiv.org/abs/1510.00149).
* **Intelligent planning**: currently we process only a single frame. How could
  more frames be taken into account?
* **Use Tegra**: Run the code on a car with the [Tegra chip set](http://www.nvidia.com/object/tegra.html).

## Git history

If you are intersted in the git history, have a look to this [repo](https://github.com/berleon/deep_car).
