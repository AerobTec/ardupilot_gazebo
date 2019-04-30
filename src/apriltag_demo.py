#!/usr/bin/env python

##
#
# Control a MAV via mavros
#
##

import rospy
import threading
from geometry_msgs.msg import Pose, PoseStamped, Twist
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import RCIn
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
from apriltags2_ros.msg import AprilTagDetectionArray

class MavController:
    """
    A simple object to help interface with mavros 
    """
    def __init__(self, namespace=""):

        #rospy.init_node("mav_control_node_")
        rospy.Subscriber(namespace + "/mavros/local_position/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber(namespace + "/mavros/rc/in", RCIn, self.rc_callback)

        self.cmd_pos_pub = rospy.Publisher(namespace + "/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher(namespace + "/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        self.rc_override = rospy.Publisher(namespace + "/mavros/rc/override", OverrideRCIn, queue_size=1)

        # mode 0 = STABILIZE
        # mode 4 = GUIDED
        # mode 9 = LAND
        self.mode_service = rospy.ServiceProxy(namespace + '/mavros/set_mode', SetMode)
        self.arm_service = rospy.ServiceProxy(namespace + '/mavros/cmd/arming', CommandBool)
        self.takeoff_service = rospy.ServiceProxy(namespace + '/mavros/cmd/takeoff', CommandTOL)

        self.rc = RCIn()
        self.pose = Pose()
        self.timestamp = rospy.Time()

    def rc_callback(self, data):
        """
        Keep track of the current manual RC values
        """
        self.rc = data

    def pose_callback(self, data):
        """
        Handle local position information
        """
        self.timestamp = data.header.stamp
        self.pose = data.pose

    def goto(self, pose):
        """
        Set the given pose as a the next setpoint by sending
        a SET_POSITION_TARGET_LOCAL_NED message. The copter must
        be in GUIDED mode for this to work. 
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.timestamp
        pose_stamped.pose = pose

        self.cmd_pos_pub.publish(pose_stamped)

    def goto_xyz(self, x, y, z):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        self.goto(pose)
    
    def get_position(self):
        my_tuple = (self.pose.position.x, self.pose.position.y, self.pose.position.z)
        return my_tuple

    def set_vel(self, vx, vy, vz, avx=0, avy=0, avz=0):
        """
        Send comand velocities. Must be in GUIDED mode. Assumes angular
        velocities are zero by default. 
        """
        cmd_vel = Twist()
        
        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.linear.z = vz

        cmd_vel.angular.x = avx
        cmd_vel.angular.y = avy
        cmd_vel.angular.z = avz

        self.cmd_vel_pub.publish(cmd_vel)

    def arm(self):
        """
        Arm the throttle
        """
        return self.arm_service(True)
    
    def disarm(self):
        """
        Disarm the throttle
        """
        return self.arm_service(False)

    def takeoff(self, height=1.0):
        """
        Arm the throttle, takeoff to a few feet, and set to guided mode
        """
        # Set to stabilize mode for arming
        mode_resp = self.mode_service(custom_mode="0")
        self.arm()

        # Set to guided mode 
        mode_resp = self.mode_service(custom_mode="4")

        # Takeoff
        takeoff_resp = self.takeoff_service(altitude=height)

        if takeoff_resp.success is not True:
            print(takeoff_resp)

        return takeoff_resp

    def land(self):
        """
        Set in LAND mode, which should cause the UAV to descend directly, 
        land, and disarm. 
        """
        resp = self.mode_service(custom_mode="9")
        self.disarm()

class ApriltagListener:
    """
    Defines an object that will listen on /tag_detections to determine the relative
    position of an AprilTag.
    """
    def __init__(self, namespace=""):
        rospy.Subscriber(namespace + "/tag_detections", AprilTagDetectionArray, self.tag_detections_callback)

        # The (last known) relative pose of the tag from the camera
        self.tag_pose = Pose()

        # The (last known) relative pose of the camera from the tag
        self.camera_pose = Pose()

        # Whether or not we can currently detect the image
        self.have_detection = False

    def tag_detections_callback(self, data):
        if data.detections:
            self.tag_pose = data.detections[0].pose.pose.pose

            # The camera relative to the tag is backwards from the tag relative to the camera
            self.camera_pose.position.x = data.detections[0].pose.pose.pose.position.y
            self.camera_pose.position.y = data.detections[0].pose.pose.pose.position.x
            self.camera_pose.position.z = data.detections[0].pose.pose.pose.position.z

            self.have_detection = True
        else:
            self.have_detection = False



def apriltag_test():
    """
    Simple demo of tracking to the center of an apriltag
    """
    searchComplete = False

    tracker = ApriltagListener()
    c = MavController()
    rospy.sleep(1)

    print("==> Takeoff")
    c.takeoff()
    rospy.sleep(10)

    # search

    xi = x0 = 0
    yi = y0 = 0
    z0 = 2

    print("==> Flying to (0,0)")
    c.goto_xyz(x0, y0, z0)
    rospy.sleep(10)

    reverse = False
    image = tracker.have_detection

    print("Start searching")
    
    while yi < 5 and not image:
        # forward search
        while not reverse:
            image = tracker.have_detection
            if image:
                break
            xi += 0.3
            c.goto_xyz(xi, yi, z0)
            rospy.sleep(0.5)
            if xi > 5:
                yi += 1.5 
                reverse = True

        # reverse search
        while reverse:
            image = tracker.have_detection
            if image:
                break
            xi -= 0.3
            c.goto_xyz(xi, yi, z0)
            rospy.sleep(0.5)
            if xi < -5:
                yi += 1.5
                reverse = False

    print("==> Regulating to center of tag")

    for i in range(50):

        xerr = tracker.camera_pose.position.x
        yerr = tracker.camera_pose.position.y

        if tracker.have_detection and xerr < 0.03 and xerr > -0.03 and yerr < 0.03 and yerr > -0.03:
            print("Search complete!")
            searchComplete = True
            break

        #print("Xerr: %s" % xerr)
        #print("Yerr: %s" % yerr)
        c.set_vel(-0.2 * xerr, -0.2 * yerr, 0)

        rospy.sleep(1)

    if searchComplete:
        print("==> Fetching target")
        x, y, z = c.get_position()
        c.goto_xyz(x, y, z-1.5)
        rospy.sleep(10)
        print("==> Flying to destination")
        c.goto_xyz(-4, -4, 2)
        rospy.sleep(15)
        print("==> Landing")
        c.land()

    if not searchComplete:
        print("Tag is not found!")
        print("==> Flying to (4,4)")
        c.goto_xyz(4,4,2)
        rospy.sleep(5)

        print("==> Landing")
        c.land()


if __name__=="__main__":
    # need to initialize a ros node before creating any MavController instances
    rospy.init_node("mav_control_node")

    apriltag_test()
