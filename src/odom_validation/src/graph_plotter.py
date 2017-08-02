import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1,projection='3d')

pullData = open("odom_car.txt","r").read()
dataArray = pullData.split('\n')
xar = []
yar = []
zar = []
for eachLine in dataArray:
    if len(eachLine)>1:
        z,x,y = eachLine.split(',')
        xar.append(float(x))
        yar.append(float(y))
        zar.append(float(z))
ax1.plot(xar,yar,zar,'b')

pullData = open("odom_camera.txt","r").read()
dataArray = pullData.split('\n')
xar = []
yar = []
zar = []
for eachLine in dataArray:
    if len(eachLine)>1:
        z,x,y = eachLine.split(',')
        xar.append(float(x))
        yar.append(float(y))
        zar.append(float(z))
ax1.plot(xar,yar,zar,'r')

plt.show()
