from __future__ import division
import rospy
import time
import os
import PIL
from scipy import misc
import tensorflow as tf
import numpy as np
from deep_car.model import Model
from deep_car.data import crop_img
from PIL import ImageDraw, ImageFont, ImageFilter
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String,Int16
from io import BytesIO



rospy.init_node('PixelDriveNN')


# # DistNN
#
# This class is responsible for predicting the steering from pictures.
# The predection is based on a neuronal network which is loaded when the class get initialised.


class DistNN:

    def __init__(self):
        self.sess = tf.InteractiveSession()

        saver = tf.train.import_meta_graph('../data/steering_model/steering_mixture_prob_exp.ckpt.meta')
        saver.restore(self.sess, tf.train.latest_checkpoint('../data/steering_model/'))

        graph = tf.get_default_graph()

        self.x_img = graph.get_tensor_by_name("deep_car/image:0")
        self.steering_abs = graph.get_tensor_by_name("deep_car/steering_abs_true:0")
        self.steering_abs_pred = graph.get_tensor_by_name("deep_car/steering_abs_pred:0")

    def predictSteering(self,img):
        steering_pred_abs, = self.sess.run(
            [ self.steering_abs_pred],
            feed_dict={
                self.x_img: img,
            }
        )
        return steering_pred_abs

nn = DistNN()



def rad2deg(rad_value):
    return (rad_value + np.pi/2) / np.pi * 180

def deg2rad(deg_value):
    return deg_value/180.0 * np.pi - np.pi/2.0


# # Car
#
# This class represents the car and gives an interface for changing the speed and steering.
# Also the callbacks for relevant ROS topics are defined in that class.
# An instance of the neuronal network is created for every Car object and and used for making predections based
# on the images from the front camera.
# The class also takes care of normalizing images for the neuronal network.


class Car:
    def __init__(self):
        self.running_framerate = None
        self.msg_timestamps = []
        self.speed = 0
        self.steering = 90
        self.current_steering = None
        self.driveNN = DistNN()
        self.dist_prob = None
        self.last_action = 0
        self.steering_pub = rospy.Publisher("/manual_control/steering", Int16, queue_size=10)
        self.speed_pub = rospy.Publisher("/manual_control/speed", Int16, queue_size=10)
        rospy.topics.Subscriber("/manual_control/speed", Int16, self.setSpeed)
        rospy.topics.Subscriber("/manual_control/steering", Int16, self.setSteering)
        rospy.topics.Subscriber("/app/camera/rgb/image_raw/compressed",
                                CompressedImage, self.processImg, queue_size=1)


    def setSpeed(self, speed):
        self.speed = speed.data

    def stop(self):
        self.pubSpeed(0)

    def driveSlow(self, direction=1):
        self.pubSpeed(direction * -150)

    def driveAverage(self, direction=1):
        self.pubSpeed(direction * -200)

    def driveFast(self, direction=1):
        self.pubSpeed(direction * -280)

    def setSteering(self, steering):
        self.steering = steering.data

    def updateFramerate(self):
        if len(self.msg_timestamps) < 2:
            return

        framerate = 1. / (self.msg_timestamps[-1] - self.msg_timestamps[-2])
        if self.running_framerate is None:
            self.running_framerate = framerate
        else:
            alpha = 0.05
            self.running_framerate *= 1 - alpha
            self.running_framerate += alpha * framerate

    def processImg(self, msg):
        img_bytes = BytesIO(msg.data)
        img_large = PIL.Image.open(img_bytes)
        img_nn = crop_img(img_large) / (255. / 2.) - 1
        img_nn = img_nn[np.newaxis, :, :, np.newaxis]
        self.msg_timestamps.append(msg.header.stamp.to_sec())
        self.updateFramerate()
        self.monitorCar()
        self.updateSteering(img_nn)

    def updateSteering(self, img):
        steering = self.driveNN.predictSteering(img)
        deg_steering = rad2deg(-steering)
        self.pubSteering(deg_steering)

    def pubSteering(self, deg_value):
        self.steering_pub.publish(deg_value)

    def getNormSteering(self):
        return np.array([[self.steering / 180.0 * np.pi - np.pi/2]])

    def pubSpeed(self, speed):
        self.speed_pub.publish(speed)

    def monitorCar(self):
        print("Framerate: ", self.running_framerate)
        print("Speed : %d | Steering : %d | Last Action : %d" %(self.speed, self.steering, self.last_action))


car = Car()
rospy.spin()
