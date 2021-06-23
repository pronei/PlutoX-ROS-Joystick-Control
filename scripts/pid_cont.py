#! /usr/bin/env python
import rospy
import numpy as np
from fly_bot_cpp.msg import kwad_input
from fly_bot_cpp.msg import kwad_state

global state #global value for drone state
global prev_time #global value for storing the time

mass = 56e-3
gravity = 9.81 

state = kwad_state() #empty initialization
control = kwad_input() #empty initialization

#callback for the position data
def StateCallback(data):
    global state
    state = data
    print("x_dot read: ", state.x_dot, "type: ", type(state.x_dot))
    print("y_dot read: ", state.y_dot, "type: ", type(state.y_dot))
    print("z_dot read: ", state.z_dot, "type: ", type(state.z_dot))

# Ros node initialization
rospy.init_node('PID_pos_hold', anonymous=True)
# Subscribe to the topic publishing position
rospy.Subscriber('/Kwad/twelve_state', kwad_state, StateCallback)
# Create a publisher for the velocities
pub = rospy.Publisher('/Kwad/control_cmd', kwad_input, queue_size=10)

Kp = np.array([20,20,10], dtype=np.float)
Kd = np.array([20,40,20], dtype=np.float)

Kp_ang = np.array([100,100,100], dtype=np.float)
Kd_ang = np.array([2,2,2], dtype=np.float)

# Rate at which to publish
rate = rospy.Rate(1)

# reference position and yaw(psi)
ref_pose = np.array([0, 0, 3])
ref_yaw = 0
ref_yaw_dot = 0

# the current time as caluclated bt ROS -> Time from the start of the ros master
prev_time = rospy.get_rostime().nsecs # get in nano seconds

# inifinite loop that runs as long as the ros master runs
while not rospy.is_shutdown():

    #Calculate errors of the x,y and z axis
    pos_error = [ref_pose[0] - state.x,
                 ref_pose[1] - state.y,
                 ref_pose[2] - state.z]
    
    curr_vel = np.array(
        [state.x_dot, state.y_dot, state.z_dot],
        dtype=np.float
    ) 

    curr_pos = np.array(
        [state.x, state.y, state.z],
        dtype=np.float
    ) 

    curr_rot = np.array(
        [state.phi, state.theta, state.psi],
        dtype=np.float
    ) 

    curr_omega = np.array(
        [state.p, state.q, state.r],
        dtype=np.float
    ) 

    # get current time
    now = rospy.get_rostime().nsecs

    # calculate time difference from the previous loop
    diff = float(now - prev_time)/1e9

    # edge case: First loop - both now and prev_time are equal, So diff = 0. Make it the one loop time = 1/rate = 1/30
    if(diff == 0):
        diff = 0.033
    
    # des_state.acc is 0
    # des_state.vel is 0

    cmd_acc = Kd * (-curr_vel) + Kp * (ref_pose - curr_pos)
    F = mass * (gravity + cmd_acc[2])

    phi_des = (1/gravity)*(cmd_acc[0]*np.sin(ref_yaw) - cmd_acc[1]*np.cos(ref_yaw))
    theta_des = (1/gravity)*(cmd_acc[0]*np.cos(ref_yaw) + cmd_acc[1]*np.sin(ref_yaw))
    rot_des = np.array([phi_des, theta_des, ref_yaw])
    omega_des = np.array([0, 0, ref_yaw_dot])
    M = Kp_ang*(rot_des - curr_pos) + Kd_ang*(omega_des - curr_omega)

    control.thrust = F
    control.tau_x = M[0]
    control.tau_y = M[1]
    control.tau_z = M[2]
    
    # publish the data
    pub.publish(control)
	
    prev_time = now
    # Set prev_error as the current_error to use in the next loop
    #prev_pose_error = pose_error
	
    # Sleep for the rate time
    rate.sleep()
