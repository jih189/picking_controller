#!/usr/bin/env python
import struct
import rospy
import roslib
from tf import TransformListener

from object_tracking_2d_ros.msg import ObjectDetection
from object_tracking_2d_ros.msg import ObjectDetections

from visualization_msgs.msg import MarkerArray

from time import sleep
import threading

from geometry_msgs.msg import(
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header 
from std_msgs.msg import String 

from baxter_core_msgs.srv import(
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface
import baxter_external_devices
from baxter_interface import CHECK_VERSION

findObject = False
movingToObject = False

def marker_callback(data):
    pass

def approach(limb_positions):
    robot = baxter_interface.RobotEnable(CHECK_VERSION)
    robot.enable()

    right_limb = baxter_interface.Limb('right')
    left_limb = baxter_interface.Limb('left')

    #angles = right_limb.joint_angles()

    right_limb.move_to_joint_positions(limb_positions)

def main_controller():
    listener = TransformListener()
    trans, rot = None, None
    while not rospy.is_shutdown():
        if movingToObject:
            listener.waitForTransform("world","megablock.obj",rospy.Time(), rospy.Duration(4.0))
            try:
                now = rospy.Time.now()
                listener.waitForTransform("world","megablock.obj",now,rospy.Duration(4.0))
                (trans,rot) = listener.lookupTransform("world","megablock.obj", now)
                break
            except Exception as e:
                print("can't get the tf because ", e)
    # after get the proper transformation of the object
    limb = 'right'
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='world')
    
    goalPose = PoseStamped(
        header=hdr,
        pose=Pose(
            position=Point(
                x=trans[0],
                y=trans[1],
                z=trans[2] + 0.1,
            ),
            orientation=Quaternion(
                x=1.0,
                y=0.0,
                z=0.0,
                w=0.0,
            ),
        ), 
    )

    ikreq.pose_stamp.append(goalPose)
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return

    # check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's

    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print "\nIK Joint Solution:\n", limb_joints
        approach(limb_joints)
        
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")


    print("done")

def direction_callback(data):
    # receive the data from ebt and detect whether they are good or not
    global findObject
    global movingToObject

    if movingToObject:
        # if it is in the moving step, then ingore all data here
        return

    if data.detections[0].good == True:
        if findObject == False:
            sleep(3) 
            findObject = True
        else:
            #go to the object
            movingToObject = True
    else:
        findObject = False

def listener():
    # set up the subscriber for ebt
    rospy.init_node('listener')
    rospy.Subscriber("/object_tracking_2d_ros/marker_array", MarkerArray, marker_callback)
    rospy.Subscriber("/object_tracking_2d_ros/detections", ObjectDetections, direction_callback, queue_size=1)
    # set up the main thread for control
    x = threading.Thread(target=main_controller)
    x.start()
    rospy.spin()


if __name__ == '__main__':
    listener()
