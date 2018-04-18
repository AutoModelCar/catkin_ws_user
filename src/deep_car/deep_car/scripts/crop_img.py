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


rospy.init_node("DeepCarCropImg")
publish_cropimg = rospy.Publisher("/deepcar/crop_img64x48/compressed", CompressedImage, queue_size=1)

def cropCallback(msg):
    time_stamp = msg.header.stamp
    img_bytes = BytesIO(msg.data)
    img = PIL.Image.open(img_bytes)
    
    crop_size = (64, 48)
    resize = (80, 60)
    img = img.convert('L').resize(resize, resample=PIL.Image.BILINEAR)
    img = img.crop(
        map(
            int,
            [ 0.5 * (resize[0] - crop_size[0]), 
              0.5 * (resize[1] - crop_size[1]),
              0.5 * (resize[0] + crop_size[0]),
              0.5 * (resize[1] + crop_size[1])])
    )
    np_img = np.asarray(img)
    
    msg = CompressedImage()
    msg.header.stamp = time_stamp
    msg.format = "png"
    msg.data = np.array(cv2.imencode('.png', np_img)[1]).tostring()
    publish_cropimg.publish(msg)
    
sub = rospy.topics.Subscriber("/app/camera/rgb/image_raw/compressed", CompressedImage, cropCallback)
rospy.spin()






