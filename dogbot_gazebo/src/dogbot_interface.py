#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import time
from unitree_legged_msgs.msg import Aliengo_Joint_controll


joint_state_read_pub= rospy.Publisher('aliengo_gazebo/joint_states', JointState, queue_size=10)
jnt_array = [0]*12
def jointState_read_callback(data):
    global jnt_array
    jnt = JointState()
    jnt.header = data.header
    jnt.name = data.name
    jnt.effort = data.effort
    jnt.velocity = data.velocity
    jnt.position = [0]*12
    jnt_array = data.position
    jnt.position[0] = -data.position[3] #1  calf  to knee    # FL  # RR
    jnt.position[1] = data.position[5] #2 hip to   roll
    jnt.position[2] = data.position[4] #0 thigh to pitch

    jnt.position[3] = data.position[0]  # FR   # RL
    jnt.position[4] = data.position[2]
    jnt.position[5] = -data.position[1]
    
    jnt.position[6] = -data.position[9] # RL # FR
    jnt.position[7] = -data.position[11]
    jnt.position[8] = data.position[10]
    
    jnt.position[9]  =  data.position[6] #RR #FL
    jnt.position[10] =  -data.position[8]
    jnt.position[11] =  -data.position[7]
    joint_state_read_pub.publish(jnt)

def jointState_write_callback(data):
    joint_pub[0].publish(-data.joint_positions[6])
    joint_pub[1].publish(-data.joint_positions[7])
    joint_pub[2].publish(data.joint_positions[8])

    joint_pub[3].publish(-data.joint_positions[9])
    joint_pub[4].publish(data.joint_positions[10])
    joint_pub[5].publish(-data.joint_positions[11])

    joint_pub[6].publish(data.joint_positions[0])
    joint_pub[7].publish(-data.joint_positions[1])
    joint_pub[8].publish(data.joint_positions[2])

    joint_pub[9].publish(data.joint_positions[3])
    joint_pub[10].publish(data.joint_positions[4])
    joint_pub[11].publish(-data.joint_positions[5])

rospy.init_node('anymal_interface', anonymous=True)
rospy.Subscriber("dogbot/joint_states", JointState, jointState_read_callback)
rospy.Subscriber("Aliengo_jnt_req_state", Aliengo_Joint_controll, jointState_write_callback)




joint_pub = [0]*12
joint_pub[0]= rospy.Publisher('/dogbot/front_left_roll_position_controller/command', Float64, queue_size=10)
joint_pub[1]= rospy.Publisher('/dogbot/front_left_pitch_position_controller/command', Float64, queue_size=10)
joint_pub[2]= rospy.Publisher('/dogbot/front_left_knee_position_controller/command', Float64, queue_size=10)
joint_pub[3]= rospy.Publisher('/dogbot/front_right_roll_position_controller/command', Float64, queue_size=10)
joint_pub[4]= rospy.Publisher('/dogbot/front_right_pitch_position_controller/command', Float64, queue_size=10)
joint_pub[5]= rospy.Publisher('/dogbot/front_right_knee_position_controller/command', Float64, queue_size=10)
joint_pub[6]= rospy.Publisher('/dogbot/back_left_roll_position_controller/command', Float64, queue_size=10)
joint_pub[7]= rospy.Publisher('/dogbot/back_left_pitch_position_controller/command', Float64, queue_size=10)
joint_pub[8]= rospy.Publisher('/dogbot/back_left_knee_position_controller/command', Float64, queue_size=10)
joint_pub[9]= rospy.Publisher('/dogbot/back_right_roll_position_controller/command', Float64, queue_size=10)
joint_pub[10]= rospy.Publisher('/dogbot/back_right_pitch_position_controller/command', Float64, queue_size=10)
joint_pub[11]= rospy.Publisher('/dogbot/back_right_knee_position_controller/command', Float64, queue_size=10)
    

stand_states = [0, 0.84, 1.32, 0, -0.84, -1.32, 0, 0.84, 1.32, 0, -0.84, -1.32]
rate = rospy.Rate(100)

def stand():
    u = 0
    t = time.time()
    while u<=1.001 and not rospy.is_shutdown():
        for i in range(12):
            joint_pub[i].publish(stand_states[i]*u + jnt_array[i]*(1-u))
        rate.sleep()
        u = (time.time() - t)/3.0

if __name__ == '__main__':
    try:    
        #jointState_read_callback(rospy.wait_for_message('dogbot/joint_states', JointState, timeout=5))
        #stand()
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass