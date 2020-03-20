#!/usr/bin/env python3
import numpy as np
import cv2, PIL, os
from cv2 import aruco
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd


imagesFolder = "src/ar_markers/src/data"

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

fig = plt.figure()
nx = 8
ny = 6
for i in range(1, nx*ny+1):
    ax = fig.add_subplot(ny,nx, i)
    img = aruco.drawMarker(aruco_dict,i-1, 700)
    plt.imshow(img, cmap = mpl.cm.gray, interpolation = "nearest")
    ax.axis("off")

plt.savefig(imagesFolder + "/markers.pdf")    
plt.show()

board = aruco.CharucoBoard_create(3, 3, 1, 0.8, aruco_dict)
imboard = board.draw((4000, 4000))
fig = plt.figure()
ax = fig.add_subplot(1,1,1)
plt.imshow(imboard, cmap = mpl.cm.gray, interpolation = "nearest")
ax.axis("off")
cv2.imwrite(imagesFolder + "/chessboard.tiff",imboard)
#plt.savefig(imagesFolder + "/chessboard.pdf")   
plt.grid()
plt.show()
print("Imprimer le damier de calibration!")

import cv2
import math

#videoFile = "E:/Desktop/S8/Projet 851/outpy.avi"
imagesFolder = "src/ar_markers/src/data_image/"
cap = cv2.VideoCapture(2)
frameRate = cap.get(5) #frame rate
while(cap.isOpened()):
    frameId = cap.get(1) #current frame number
    ret, frame = cap.read()
    #if (ret != True):
    if (frameId == 150):
        print("break")
        break
    if (frameId <150):
        filename = imagesFolder + "/image_" +  str(int(frameId)) + ".jpg"
        cv2.imwrite(filename, frame)
cap.release()
print ("Done!")

im = PIL.Image.open(imagesFolder + "/image_0.jpg")
fig = plt.figure()
ax = fig.add_subplot(1,1,1)
plt.imshow(im)
ax.axis('off')
plt.show()

def read_chessboards(images):
    """
    Charuco base pose estimation.
    """
    print("POSE ESTIMATION STARTS:")
    allCorners = []
    allIds = []
    decimator = 0
    
    for im in images:
        print("=> Processing image {0}".format(im))
        frame = cv2.imread(im)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        res = cv2.aruco.detectMarkers(gray, aruco_dict)

        if len(res[0])>0:
            res2 = cv2.aruco.interpolateCornersCharuco(res[0],res[1],gray,board)        
            if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%1==0:
                allCorners.append(res2[1])
                allIds.append(res2[2])              

        decimator+=1   

    imsize = gray.shape
    return allCorners,allIds,imsize
    print("finished")

images = [imagesFolder + f for f in os.listdir(imagesFolder) if f.startswith("image_")]
allCorners,allIds,imsize=read_chessboards(images)


def calibrate_camera(allCorners,allIds,imsize):   
    """
    Calibrates the camera using the dected corners.
    """
    print("CAMERA CALIBRATION")
    
    cameraMatrixInit = np.array([[ 2000.,    0., imsize[0]/2.],
                                 [    0., 2000., imsize[1]/2.],
                                 [    0.,    0.,           1.]])

    distCoeffsInit = np.zeros((5,1))
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL) 
    (ret, camera_matrix, distortion_coefficients0, 
     rotation_vectors, translation_vectors,
     stdDeviationsIntrinsics, stdDeviationsExtrinsics, 
     perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
                      charucoCorners=allCorners,
                      charucoIds=allIds,
                      board=board,
                      imageSize=imsize,
                      cameraMatrix=cameraMatrixInit,
                      distCoeffs=distCoeffsInit,
                      flags=flags,
#                      criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.001))
                      criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))
    return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors
    print("finished")


ret, mtx, dist, rvecs, tvecs = calibrate_camera(allCorners,allIds,imsize)
ret

mtx

dist
#create a file to store data
from lxml import etree
from lxml.builder import E
path = os.path.abspath('.')
fname = path + "/calibration_result/calibration_parameters.txt"
print(fname)
with open(fname, "w") as f:
    f.write("{'ret':"+str(ret)+", 'mtx':"+str(list(mtx))+', "dist":'+str(list(dist))+'}')
    f.close()
np.savetxt(imagesFolder+"calib_mtx_webcam.csv", mtx)
np.savetxt(imagesFolder+"calib_dist_webcam.csv", dist)


#imagesFolder = "data_image_detect/"

i=24 # select image id
plt.figure()
frame = cv2.imread(imagesFolder + "image_100.jpg".format(i))
img_undist = cv2.undistort(frame,mtx,dist,None)
plt.subplot(211)
plt.imshow(frame)
plt.title("Raw image")
plt.axis("off")
plt.subplot(212)
plt.imshow(img_undist)
plt.title("Corrected image")
plt.axis("off")
plt.show()

##Use of camera calibration to estimate 3D translation and rotation of each marker on a scene
imagesFolder = "src/ar_markers/src/data_image_detect/"
frame = cv2.imread(imagesFolder + "image_10.png")
plt.figure()
plt.imshow(frame)
plt.show()

print(1111111111111111)

##Post processing
#%%time

gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters =  aruco.DetectorParameters_create()
corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

#Result
conn = np.array([0, 1, 2, 3, 0])
plt.figure()
plt.imshow(frame_markers)
plt.legend()
plt.show()

#Add local axis on each maker
size_of_marker =  0.0145 # side lenght of the marker in meter
rvecs,tvecs, trash = aruco.estimatePoseSingleMarkers(corners, size_of_marker , mtx, dist)

tvecs

tvecs.shape

np.degrees(rvecs)

length_of_axis = 0.01
imaxis = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
for i in range(len(tvecs)):
    imaxis = aruco.drawAxis(imaxis, mtx, dist, rvecs[i], tvecs[i], length_of_axis)

plt.figure()
plt.imshow(imaxis)
plt.show()


#AR_marker data
data=pd.DataFrame(data=tvecs.reshape(43,3),columns=["tx","ty","tz"],index=ids.flatten())
data.index.name="makers"
data.sort_index(inplace=True)
data

p=data.values
((p[1]-p[0])**2.).sum()**.5,((p[2]-p[1])**2.).sum()**.5,((p[3]-p[2])**2.).sum()**.5

((data.loc[11]-data.loc[0]).values**2).sum()

V0_1= p[1]-p[0]
V0_11=p[11]-p[0]
V0_1,V0_11

np.dot(V0_1,V0_11)

fig=plt.figure()
ax= fig.add_subplot(1,1,1)
ax.set_aspect("equal")
plt.plot(data.tx[:10], data.ty[:10],"or-")
plt.grid()
plt.show()


data.tx

corners=np.array(corners)
data2=pd.DataFrame({"px":corners[:,0,0,1],"py":corners[:,0,0,0]},index=ids.flatten())
data2.sort_index(inplace=True)

data2

n0=data2.loc[0]
n1=data2.loc[1]
d01=((n0-n1).values**2).sum()**.5
d=42.5e-3
factor=d/d01
data2["x"]=data2.px*factor
data2["y"]=data2.py*factor
d1_0=data2.loc[2].y-data2.loc[1].y
d11_0=data2.loc[11].x-data2.loc[0].x
d1_0

d11_0

imagesFolder = "/src/ar_markers/src/image_result/"
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
board = cv2.aruco.CharucoBoard_create(3,3,.025,.0125,dictionary)
img = board.draw((200*3,200*3))
cv2.imwrite(imagesFolder + '/charucotest.png',img)
