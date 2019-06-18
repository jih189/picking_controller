#!/usr/bin/env python
import rospy
import roslib
from std_msgs.msg import String
from tf import TransformListener

from geometry_msgs.msg import Pose
from object_tracking_2d_ros.msg import ObjectDetection
from object_tracking_2d_ros.msg import ObjectDetections

from visualization_msgs.msg import MarkerArray

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION

from time import sleep
import threading

findObject = False
movingToObject = False

def marker_callback(data):
    pass

def moveToObject(position):
    tf = TransformListener()
    print("all frames")
    print(tf.getFrameStrings())
    if tf.frameExists("/base"):
        (pos, quaternion) = tf.lookupTransform("/base","/megablock.obj", rospy.Time(0)) 
        print("base position is ", pos)
    else:
        print("frame does not exists\n")


def testing_listener():
    listener = TransformListener()
    while not rospy.is_shutdown():
        if movingToObject:
            #print(tf.getFrameStrings())
            listener.waitForTransform("base","megablock.obj",rospy.Time(), rospy.Duration(4.0))
            try:
                now = rospy.Time.now()
                listener.waitForTransform("base","megablock.obj",now,rospy.Duration(4.0))
                (trans,rot) = listener.lookupTransform("base","megablock.obj", now)
                print(trans)
            except Exception as e:
                print("can't get the tf because ", e)

def direction_callback(data):
    global findObject
    global movingToObject
    if movingToObject:
        return
    print("detecting..")
    #print(data.detections[0].good)
    if data.detections[0].good == True:
        if findObject == False:
            #sleep(10) 
            findObject = True
        else:
            #go to the object
            movingToObject = True
            #moveToObject(data.detections[0].pose)
    else:
        findObject = False

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/object_tracking_2d_ros/marker_array", MarkerArray, marker_callback)
    rospy.Subscriber("/object_tracking_2d_ros/detections", ObjectDetections, direction_callback, queue_size=1)
    x = threading.Thread(target=testing_listener)
    x.start()
    rospy.spin()

def testing():
    right = baxter_interface.Limb('right')
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    rj = right.joint_names()
    
    current_position = right.joint_angle(rj[6])
    joint_command = {rj[6]: current_position - 1.0}
    right.move_to_joint_positions(joint_command)
    joint_command = {rj[6]: current_position}
    right.move_to_joint_positions(joint_command)


if __name__ == '__main__':
    '''
    print("Initializing node... ")
    rospy.init_node("pick_and_place_controller")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot... ")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)
    print("Enabling robot... ")
    rs.enable()
    #testing()
    print("done")
    '''
    listener()
