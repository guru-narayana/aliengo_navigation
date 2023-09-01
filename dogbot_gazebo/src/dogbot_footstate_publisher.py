#!/usr/bin/env python3


import rospy
from aliengo_msgs.msg import quad_footstep
from gazebo_msgs.msg import ContactsState
from math import sqrt 
import tf


foot_status = quad_footstep()
foot_status.contact_state = [0]*4

def FL_contact_cb(data):
    global foot_status
    if(len(data.states) == 0):
        foot_status.contact_state[0] = 0
        return
    force = sqrt(pow(data.states[0].total_wrench.force.x,2)+pow(data.states[0].total_wrench.force.y,2)+pow(data.states[0].total_wrench.force.z,2))
    if(force>1):
        foot_status.contact_state[0] = 1
    else:
        foot_status.contact_state[0] = 0
def FR_contact_cb(data):
    global foot_status
    if(len(data.states) == 0):
        foot_status.contact_state[1] = 0
        return
    force = sqrt(pow(data.states[0].total_wrench.force.x,2)+pow(data.states[0].total_wrench.force.y,2)+pow(data.states[0].total_wrench.force.z,2))
    if(force>1):
        foot_status.contact_state[1] = 1
    else:
        foot_status.contact_state[1] = 0
def RL_contact_cb(data):
    global foot_status
    if(len(data.states) == 0):
        foot_status.contact_state[2] = 0
        return
    force = sqrt(pow(data.states[0].total_wrench.force.x,2)+pow(data.states[0].total_wrench.force.y,2)+pow(data.states[0].total_wrench.force.z,2))
    if(force>1):
        foot_status.contact_state[2] = 1
    else:
        foot_status.contact_state[2] = 0
def RR_contact_cb(data):
    global foot_status
    if(len(data.states) == 0):
        foot_status.contact_state[2] = 0
        return
    force = sqrt(pow(data.states[0].total_wrench.force.x,2)+pow(data.states[0].total_wrench.force.y,2)+pow(data.states[0].total_wrench.force.z,2))
    if(force>1):
        foot_status.contact_state[3] = 1
    else:
        foot_status.contact_state[3] = 0


rospy.init_node('footstep_state_publisher', anonymous=True)


foot_pub = rospy.Publisher('Aliengo_foot_crntstate', quad_footstep, queue_size=10)

rospy.Subscriber("/dogbot/back_right_foot_contactsensor_state", ContactsState, FL_contact_cb)
rospy.Subscriber("/dogbot/back_left_foot_contactsensor_state", ContactsState, FR_contact_cb)
rospy.Subscriber("/dogbot/front_right_foot_contactsensor_state", ContactsState, RL_contact_cb)
rospy.Subscriber("/dogbot/front_left_foot_contactsensor_state", ContactsState, RR_contact_cb)
listener = tf.TransformListener()
rate = rospy.Rate(200)

while not rospy.is_shutdown():
    try:
        (LF_trans,_) = listener.lookupTransform('/base', '/FL_foot', rospy.Time(0))
        (RF_trans,_) = listener.lookupTransform('/base', '/FR_foot', rospy.Time(0))
        (LH_trans,_) = listener.lookupTransform('/base', '/RL_foot', rospy.Time(0))
        (RH_trans,_) = listener.lookupTransform('/base', '/RR_foot', rospy.Time(0))
        (LF_wtrans,_) = listener.lookupTransform('/map', '/FL_foot', rospy.Time(0))
        (RF_wtrans,_) = listener.lookupTransform('/map', '/FR_foot', rospy.Time(0))
        (LH_wtrans,_) = listener.lookupTransform('/map', '/RL_foot', rospy.Time(0))
        (RH_wtrans,_) = listener.lookupTransform('/map', '/RR_foot', rospy.Time(0))  

        foot_status.FL.x = LF_trans[0]
        foot_status.FL.y = LF_trans[1]
        foot_status.FL.z = LF_trans[2]
        foot_status.FR.x = RF_trans[0]
        foot_status.FR.y = RF_trans[1]
        foot_status.FR.z = RF_trans[2]
        foot_status.RL.x = LH_trans[0]
        foot_status.RL.y = LH_trans[1]
        foot_status.RL.z = LH_trans[2]
        foot_status.RR.x = RH_trans[0]
        foot_status.RR.y = RH_trans[1]
        foot_status.RR.z = RH_trans[2]

        foot_status.FLw.x = LF_wtrans[0]
        foot_status.FLw.y = LF_wtrans[1]
        foot_status.FLw.z = LF_wtrans[2]
        foot_status.FRw.x = RF_wtrans[0]
        foot_status.FRw.y = RF_wtrans[1]
        foot_status.FRw.z = RF_wtrans[2]
        foot_status.RLw.x = LH_wtrans[0]
        foot_status.RLw.y = LH_wtrans[1]
        foot_status.RLw.z = LH_wtrans[2]
        foot_status.RRw.x = RH_wtrans[0]
        foot_status.RRw.y = RH_wtrans[1]
        foot_status.RRw.z = RH_wtrans[2]
        
        foot_pub.publish(foot_status)
        rate.sleep()
    
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
