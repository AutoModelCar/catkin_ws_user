#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import pickle
import sys
import rospy
import cv2
import numpy as np
from math import isnan, atan, cos, sin, pi, atan2
from sklearn import linear_model
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from cv_bridge import CvBridge, CvBridgeError
import tf

# from __future__ import print_function

xt_list = [[0]*2]*2
sigmat_list = [[[0]*2]*2]*2 


class image_converter:
  def __init__(self):
    self.image_pub = rospy.Publisher("/image_processing/usb_cam_s", Image, queue_size=1)
    self.gps_pub = rospy.Publisher("/visual_gps/odom", Odometry, queue_size = 10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_rect_color",Image,self.callback, queue_size=1)
    self.yaw_sub = rospy.Subscriber("/model_car/yaw", Float32, self.yaw_callback, queue_size=1) 
    #self.speed_sub = rospy.Subscriber("/model_car"

  def yaw_callback(self, yaw):
    self.yaw = yaw

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    ### BUILD RGB ###
    b,g,r = cv2.split(cv_image)
    r_blur = cv2.GaussianBlur(r,(5,5),0)
    g_blur = cv2.GaussianBlur(g,(5,5),0)
    b_blur = cv2.GaussianBlur(b,(5,5),0)



    #### BUILD HSV ###
    #hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV);
    #_,s,_ = cv2.split(hsv)
    #s_blur = cv2.GaussianBlur(s,(5,5),0)

    #### S BINARY
    #bi_s_max = 255
    #bi_s_min = 140
    #_, s_binary = cv2.threshold(s_blur,
                              #bi_s_min,
                              #bi_s_max,
                              #cv2.THRESH_BINARY);



    ### BUILD YUV ###
    yuv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2YUV);
    y,u,v = cv2.split(yuv)
    v_blur = cv2.GaussianBlur(v,(5,5),0)


    #Testen, in welchen Quadraten die Ballons sitzen ( im Testbag)
    #hoehe = 70
    #breite = 300
    #image = cv2.line(s_binary, tuple([ 0, hoehe ]), tuple([479, hoehe]), (255,150,70),2)
    #image = cv2.line(s_binary, tuple([ 0, hoehe + 50 ]), tuple([479, hoehe + 50]), (255,150,70),2)
    #image = cv2.line(s_binary, tuple([  breite,0 ]), tuple([breite, 479]), (255,150,70),2)
    #image = cv2.line(s_binary, tuple([  breite + 50 ,0]), tuple([breite+ 50, 479]), (255,150,70),2)

    #image = cv2.line(image, tuple([int(line2[0]), 120]), tuple([int(line2[1]), 479]), 1, thickness =$





    ### FINDING COLOR RANGES FOR THE DIFFERENT BALLOONS ###
    #starting_points = [[210,70], [300,70 ], [210, 200], [300, 200]]


    #color_array = [] # rot, lila, blau, gruen
    ##Zeilen sind Farben:
    #mean_array = np.zeros((4,4))
    #std_array = np.zeros((4, 4))
    #for k,spk in enumerate(starting_points):
      #color_array.append([])
      #for j in range(spk[0], spk[0]+50):
        #for i in range(spk[1], spk[1]+50):
          #if s_binary[i,j] == 255:
            #color_array[k].append([r[i,j], g[i,j], b[i,j], v[i,j]])

      #mean_array[k] = np.mean(color_array[k], axis = 0)
      #std_array[k] = np.std(color_array[k], axis = 0)
    #print(mean_array, std_array)

    mean_array = np.array([
        [ 110,   36,   51 ,  172],
        [  60,   31,  175,  131 ],
        [  24,   27,  184,  110],
        [  19,   85,   32,   92]
        ])
    std_array = np.array([
        [ 16,  11 ,  11,   6],
        [ 38,  22,  76,   7],
        [ 11,  11,  80,   8],
        [  9,  24,  13,   9]
        ])



    # define range of balloons in RGB
    lower_rgb = np.array([[94, 25, 40], #r, g, b
                         [22, 9 , 99],
                         [13, 16, 104],
                         [10, 61, 19]])
    lower_rgb = np.fliplr(lower_rgb) # flip to b, g, r

    upper_rgb = np.array([[126,47, 62], # r, g, b
                         [98, 53, 251],
                         [35, 38, 255],
                         [28, 109, 45]])
    upper_rgb = np.fliplr(upper_rgb) # flip to b, g, r

    lower_v = np.array([[166],
                       [124],
                       [102],
                       [83]])
    upper_v = np.array([[178],
                       [138],
                       [118],
                       [101]])

    image = cv_image
    #images = {}
    balloons_seen = [0]*4
    balloon_pos_im = [0]*4

    for k in range(4):
        mask_rgb = cv2.inRange(cv_image, lower_rgb[k], upper_rgb[k])
        mask_v   = cv2.inRange(v  , lower_v[k], upper_v[k])
        just_balloon = cv2.bitwise_and(mask_rgb, mask_v)
        #images[k] = cv2.bitwise_and(mask_rgb, mask_v)
        indices = np.nonzero(just_balloon)
        balloon_pos_im[k] = [ np.mean(indices[0]), np.mean(indices[1]) ]
        if not isnan(balloon_pos_im[k][0]): # check, if balloon appeared in picture
            balloons_seen[k] =1
            image = cv2.circle(image, tuple([ int(balloon_pos_im[k][1]) ,int( balloon_pos_im[k][0]) ]), 6, upper_rgb[k], 2)
        #print(balloon_pos_im[k])
#    print(balloons_seen)
#    print( 'gruener Ballon: ' + str(balloon_pos_im[3]))
    ### Testen, ob Farberkennung funktioniert:
    #max_length = [8, 40, 30, 10]
    #binary = r
    #balloon_pos_im = np.zeros((4,2))
    #for k in range(4): #Ballon
        #colored_points_k = []
        #for i in range(480): # Zeile
            #if len(colored_points_k) == max_length[k]:
                #break
            #for j in range(640): # Spalte
                #binary[i,j]=0
                #if len(colored_points_k) == max_length[k]:
                    #break
                #if mean_array[k][0] - std_array[k][0] <= r_blur[i,j] <= mean_array[k][0] + std_array[k][0]: # Rotwert
                    #if mean_array[k][1] - std_array[k][1] <= g_blur[i,j] <= mean_array[k][1] + std_array[k][1]: #Gruenwert
                        #if mean_array[k][2] - std_array[k][2] <= b_blur[i,j] <= mean_array[k][2] + std_array[k][2]: #Blauwert
                            #if mean_array[k][3] - std_array[k][3] <= v_blur[i,j] <= mean_array[k][3] + std_array[k][3]: #V-Wert (von YUV)
                                #binary[i,j] = 50+(k+1)*50
                                #colored_points_k.append([i,j])
        ##print('laenge von Liste, die zu Farbe k gehoert: ' + str(len(colored_points_k)))
        #balloon_pos_im[k] = np.mean(np.array(colored_points_k), axis = 0)


    balloon_pos_rw = [
        [3.57, -3    ],
        [2.33, -3    ],
        [3.57, -1.15 ],
        [2.33, -1.15 ]]

    #centers of the four balloons
    center_im = np.mean(
        np.array([ balloon_pos_im[k] for k in range(4) if balloons_seen[k] ]),
        axis=0
        )
    #image = cv2.circle(image, tuple([ int(center_im[1]) ,int( center_im[0])]), 3, (0,0,0), 2)
#    print('center im ', center_im)
    center_rw = np.mean(
        np.array([ balloon_pos_rw[k] for k in range(4) if balloons_seen[k] ]),
        axis=0)
#    print('center  rw ', center_rw)

    # image = cv2.line(v, tuple([ balloon_pos_im[0][1] , balloon_pos_im[0][0] ]), tuple([ balloon_pos_im[1][1] , balloon_pos_im[1][0] ]), (255,150,70), 2)
    # image = cv2.line(image, tuple([ balloon_pos_im[2][1] , balloon_pos_im[2][0] ]), tuple([ balloon_pos_im[3][1] , balloon_pos_im[3][0] ]), (255,150,70), 2)

    # calculate rotation matrix
    scaling = []
    rotation_angle = []
    sum_x=0
    sum_y=0
    for k in range(4):
        if balloons_seen[k]:
            scaling.append( np.sqrt(  ((balloon_pos_rw[k][0]-center_rw[0])**2+( balloon_pos_rw[k][1]-center_rw[1] )**2) / float(((balloon_pos_im[k][0]-center_im[0])**2+( balloon_pos_im[k][1]-center_im[1] )**2)) ) )

            atan_1 = atan2( (balloon_pos_rw[k][1] - center_rw[1] ), (balloon_pos_rw[k][0] - center_rw[0]) )
            #atan1 = atan( (balloon_pos_rw[k][1] - center_rw[1] )/ (balloon_pos_rw[k][0] - center_rw[0]) )
            #if balloon_pos_rw[k][0] - center_rw[0] <0:
            #    atan1 +=np.sign(balloon_pos_rw[k][1] - center_rw[1]) * pi
            atan_2 = atan2( (balloon_pos_im[k][1] - center_im[1]), (balloon_pos_im[k][0] - center_im[0]) )
            #atan2 = atan( (balloon_pos_im[k][1] - center_im[1])/ (balloon_pos_im[k][0] - center_im[0]) )
            #if balloon_pos_im[k][0] - center_im[0] <0:
            #    atan2 += pi * np.sign(balloon_pos_im[k][1] - center_im[1])

            #modulo 2*pi:
            rotation_angle.append( atan_1 - atan_2 )
            b_x=balloon_pos_im[k][0] - center_im[0]
            b_y=balloon_pos_im[k][1] - center_im[1]
            w_x=balloon_pos_rw[k][0] - center_rw[0]
            w_y=balloon_pos_rw[k][1] - center_rw[1]
            sum_x+=(b_x*w_y-b_y*w_x)
            sum_y+=b_x*w_x+b_y*w_y
            if rotation_angle[-1] > 2*pi:
                rotation_angle[-1] -= 2*pi
            if rotation_angle[-1] < 0:
                rotation_angle[-1] += 2*pi

    print(balloons_seen)
    scaling_mean = np.mean( scaling)
    rotation_angle_mean = np.mean( rotation_angle[1:], axis = 0 )
    rotation_angle_mean = atan2(sum_x,sum_y)

    print( 'angle: ' + str(rotation_angle))
    print( 'scaling: ' + str(scaling_mean))

    R = scaling_mean * np.array([
        [cos(rotation_angle_mean), -sin(rotation_angle_mean)],
        [sin(rotation_angle_mean),  cos(rotation_angle_mean)]
        ])

    # current_pos_im = [226, 317]
    current_pos_im = [240, 320]
    current_pos_rw = np.dot(R, current_pos_im - center_im) + center_rw

    
    #### Kalman-Filter ###
    #global sigmat_list
    #global xt_list
    #var_measurement = np.array([[0.19, 0], [0,0.46]])
    #sigmat_list[0] = sigmat_list[1]
    #xt_list[0] = xt_list[1] # das ist nun x_{t-1}, also x_t aus dem vorherigen Schritt
   

    ##Forecast
    #x_bar = np.dot(P, xt_list[0]) + vel * np.array([ cos(yaw) , sin(yaw)])


    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
    except CvBridgeError as e:
      print(e)
      
#    if flag == 0:
#      with open("lane_picture", "w") as f:
#        pickle.dump((rgb_binary, hsv_binary, yuv_binary), f)
#        flag = 1

    odom1 = Odometry()
    odom1.header.frame_id  = 'map'
    odom1.header.seq = data.header.seq
    odom1.pose.pose.position.x = current_pos_rw[0]
    odom1.pose.pose.position.y = -current_pos_rw[1]

    # [x,y] = np.dot( R, np.array([-1, 0]) )
    # if x  == 0:
    #     alpha = pi * np.sign( y )
    # else:
    #     alpha = atan( y / float(x) )
    # if x <0:
    #     alpha += pi
    # # modulo 2*pi:
    # if alpha > 2*pi:
    #     alpha -= 2*pi
    # if alpha  < 0:
    #     alpha += 2*pi
    # print(alpha)
    # if sum(balloons_seen) >= 3:
    odom1.pose.pose.orientation = Quaternion( *tf.transformations.quaternion_from_euler(0, 0, pi-1.0*rotation_angle_mean ) )
    # odom.pose.pose.orientation = Quaternion([ alpha, 0,0,0])
    if sum(balloons_seen) >= 3:
    	self.gps_pub.publish( odom1 )

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
  main(sys.argv)


