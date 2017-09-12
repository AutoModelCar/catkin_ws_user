import rospy
import time
import os
import PIL
import cv2
import numpy as np
from scipy import misc
from PIL import ImageDraw, ImageFont, ImageFilter
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String,Int16
from io import BytesIO


rospy.init_node("DeepCarResizeImg")
publish_cropimg = rospy.Publisher("/deepcar/resize_img80x60/compressed", CompressedImage, queue_size=1)

def cropCallback(msg):
    time_stamp = msg.header.stamp
    img_bytes = BytesIO(msg.data)
    img = PIL.Image.open(img_bytes)
    
    resize = (80, 60)
    img = img.resize(resize, resample=PIL.Image.BILINEAR)
    np_img = np.asarray(img)
    
    msg = CompressedImage()
    msg.header.stamp = time_stamp
    msg.format = "png"
    msg.data = np.array(cv2.imencode('.png', np_img)[1]).tostring()
    publish_cropimg.publish(msg)
    
sub = rospy.topics.Subscriber("/app/camera/rgb/image_raw/compressed", CompressedImage, cropCallback)
rospy.spin()






