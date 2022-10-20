#!/usr/bin/env python3

import rospy
import signal
import sys

import ur_dashboard_msgs.srv
import std_srvs.srv
import configuration_msgs.srv
import std_msgs.msg
import actionlib
import relative_cartesian_controller_msgs.msg
from pathlib import Path

import tf


cont = True
def handler(signal, frame):
    global cont
    cont = False


from math import pi, cos, sin, sqrt, atan2

def norm2(a, b, c=0.0):
  return sqrt(a**2 + b**2 + c**2)

def quat_to_ur_axis_angle(quaternion):
  # https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation#Unit_quaternions
  # quaternion must be [xyzw]
  angle = 2*atan2(norm2(quaternion[0], quaternion[1], quaternion[2]), quaternion[3])
  if abs(angle) > 1e-6:
    axis_normed = [ quaternion[0]/sin(angle/2), quaternion[1]/sin(angle/2), quaternion[2]/sin(angle/2) ]
  else:
    axis_normed = [0.0, 0.0, 0.0]
  return [axis_normed[0]*angle, axis_normed[1]*angle, axis_normed[2]*angle]

class RelativeMove(object):
    # create messages that are used to publish feedback/result
    _feedback = relative_cartesian_controller_msgs.msg.RelativeMoveFeedback()
    _result = relative_cartesian_controller_msgs.msg.RelativeMoveResult()
    _listener = []
    _template=""

    _stop_srv=[]
    _play_srv=[]
    _state_srv=[]
    _pub=[]

    def __init__(self,name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, relative_cartesian_controller_msgs.msg.RelativeMoveAction, execute_cb=self.execute_cb, auto_start = False)
        self._listener = tf.TransformListener()
        self._template = Path('template.urscript').read_text()

        self._stop_srv  = rospy.ServiceProxy('/ur10e_hw/dashboard/stop', std_srvs.srv.Trigger)
        self._play_srv  = rospy.ServiceProxy('/ur10e_hw/dashboard/play', std_srvs.srv.Trigger)
        self._state_srv = rospy.ServiceProxy('/ur10e_hw/dashboard/program_state', ur_dashboard_msgs.srv.GetProgramState)
        self._pub = rospy.Publisher('/ur10e_hw/script_command', std_msgs.msg.String, queue_size=10)

        self._as.start()
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(500)
        success = True

        self._stop_srv()
        rospy.sleep(0.5)
        try:
            (trans,rot) = self._listener.lookupTransform('/base_link', goal.relative_pose.header.frame_id, rospy.Time(0))
            print("tran = ",trans,", rot = ", rot)
            frame=trans+ quat_to_ur_axis_angle(rot)

            displacement_trans=[goal.relative_pose.pose.position.x,goal.relative_pose.pose.position.y,goal.relative_pose.pose.position.z]
            displacement_quat=[goal.relative_pose.pose.orientation.x, goal.relative_pose.pose.orientation.y, goal.relative_pose.pose.orientation.z, goal.relative_pose.pose.orientation.w]
            displacement=displacement_trans+quat_to_ur_axis_angle(displacement_quat)
            print(frame)
            print(displacement)

            command=self._template
            command=command.replace("FRAME_VALUE",str(frame))
            command=command.replace("DISPLACEMENT_VALUE",str(displacement))
            command=command.replace("ACC",str(1.2))
            vel=goal.target_linear_velocity
            if vel==0:
                vel=0.05
            command=command.replace("VEL",str(vel))


            self._pub.publish(command)

            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            success=False
            self._as.set_failed(self._result)
        #self._feedback.sequence = []

        rospy.sleep(0.5)
        while True:
            lg_state=self._state_srv();
            if lg_state.state.state=="STOPPED":
                break
        self._play_srv()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, handler)
    rospy.init_node('relative_move')
    RelativeMove(rospy.get_name())
    rospy.spin()
