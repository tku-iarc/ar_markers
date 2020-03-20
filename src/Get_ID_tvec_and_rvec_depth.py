#!/usr/bin/env python3
import os
import threading
import rospy
import numpy as np
import cv2
import cv2.aruco as aruco
import pyrealsense2 as rs
from std_msgs.msg import String
from ar_markers.srv import *
#from ar_markers.msg import *
from std_msgs.msg import Int32MultiArray


##-----------switch define------------##
class switch(object):
    def __init__(self, value):
        self.value = value
        self.fall = False

    def __iter__(self):
        """Return the match method once, then stop"""
        yield self.match
        raise StopIteration

    def match(self, *args):
        """Indicate whether or not to enter a case suite"""
        if self.fall or not args:
            return True
        elif self.value in args: # changed for v1.5, see below
            self.fall = True
            return True
        else:
            return False

##--------ar markers server--------###
def AR_data(req):
    ack = '%s'%req.ack

    id_sent = ids    
    tvec_sent = tvec
    rvec_sent = rvec
    return(id_sent,tvec_sent,rvec_sent)
def ar_server():
    a = rospy.Service('ar_mode',ar_data, AR_data) ##server ar data
    print ("Ready to connect")
    rospy.spin() ## spin one

def myhook():
    print ("shutdown time!")

def realsense_depth():
    # pipeline = rs.pipeline()
    # config = rs.config()
    # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    # pipeline.start(config)
    
    # try:
    #     while True:
    #         frames = pipeline.wait_for_frames()
    #         depth_frame = frames.get_depth_frame()
    #         color_frame = frames.get_color_frame()
    #         if not depth_frame or not color_frame:
    #             continue
    
    #         depth_image = np.asanyarray(depth_frame.get_data())
    #         color_image = np.asanyarray(color_frame.get_data())
    
    #         depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_WINTER)

    # ################加入這段程式#####################

    #         print("shape of color image:{0}".format(color_image.shape))
    #         print("shape of depth image:{0}".format(depth_colormap.shape))
    #         print("depth value in m:{0}".format(depth_frame.get_distance(320, 240)))


    #         text_depth = "depth value of point (320,240) is "+str(np.round(depth_frame.get_distance(320, 240),4))+"meter(s)"
    #         color_image = cv2.circle(color_image,(320,240),1,(0,255,255),-1)
    #         color_image=cv2.putText(color_image, text_depth, (10,20),  cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255), 1, cv2.LINE_AA)

    # #################################################

    #         images = np.hstack((color_image, depth_colormap))
            
    #         cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    #         cv2.imshow('RealSense', images)
    
    
    #         key = cv2.waitKey(1)
    #         if key & 0xFF == ord('q') or key == 27:
    #             cv2.destroyAllWindows()
    #             break
    
    
    # finally:
    #     pipeline.stop()
    print("thread")

if __name__ == '__main__':
    
    argv = rospy.myargv()
    rospy.init_node('ar_ros_server', anonymous=True)

    # t = threading.Thread(target=realsense_depth)
    # t.start() # 開啟多執行緒


#test wheater already calibrated or not
    path = os.path.abspath('./src/ar_markers')
#path = os.path.dirname("./ar_markers/")
    fname = path + "/res/calibration_parameters.txt"
    print(fname)
    try:
        f = open(fname, "r")
        f.read()
        f.close()
    except:
        print("You have not imported the calibration file")
  

    cap = cv2.VideoCapture(2)

#importing aruco dictionary
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

#calibration parameters
    f = open(fname, "r")
    ff = [i for i in f.readlines()]
    f.close()
    from numpy import array
    parameters = eval(''.join(ff))
    mtx = array(parameters['mtx'])
    dist = array(parameters['dist'])

# Create absolute path from this module
    file_abspath = os.path.join(os.path.dirname(__file__), 'Samples/box.obj')


    ids  = None
    tvec = [[[0, 0, 0]]]
    rvec = [[[0, 0, 0]]]

# ready connect client - arm 
    #ar_server()

    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250 )
    markerLength = 0.25   # Here, our measurement unit is centimetre.
    parameters = cv2.aruco.DetectorParameters_create()
    parameters.adaptiveThreshConstant = 10

    while True:

        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        font = cv2.FONT_HERSHEY_SIMPLEX
        if np.all(ids != None):
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)

        #print(ids)
            #print(corners)
        #print(rvec)
            for i in range(0, ids.size):
                aruco.drawAxis(frame, mtx, dist, rvec[i], tvec[i], 0.1)

            # show translation vector on the corner
                font = cv2.FONT_HERSHEY_SIMPLEX
                text = str([round(i,5) for i in tvec[i][0]])
                position = tuple(corners[i][0][0])
                cv2.putText(frame, text, position, font, 0.4, (0, 0, 0), 1, cv2.LINE_AA)

                #get tvec, rvec of each id
                # print(ids[i])
                #print(rvec[i])
                print(tvec[i][0])
                #print(rvec[i][0])

                print(position)
            # print(ids)
            # print(tvec)
            # print(rvec)
            #print(dist)

            aruco.drawDetectedMarkers(frame, corners)

        else:
            tvec = [[[0, 0, 0]]]
            rvec = [[[0, 0, 0]]]
        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # t.join()
    cap.release()
    cv2.destroyAllWindows()
    rospy.on_shutdown(myhook)
    #rospy.spin()
