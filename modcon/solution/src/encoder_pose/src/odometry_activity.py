#!/usr/bin/env python
# coding: utf-8

# In[16]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

#TODO: write a correct function

def DeltaPhi(encoder_msg, prev_ticks):
    """
        Args:
            encoder_msg: ROS encoder message (ENUM)
            prev_ticks: Previous tick count from the encoders (int)
        Return:
            rotation_wheel: Rotation of the wheel in radians (double)
            ticks: current number of ticks (int)
    """
    
    # TODO: these are random values, you have to implement your own solution in here
    ticks = encoder_msg.data
    
    N_tot = encoder_msg.resolution 
    alpha = 2 * 3.141592653589793 / N_tot 
    delta_ticks = ticks - prev_ticks
    delta_phi = delta_ticks * alpha 

    return delta_phi, ticks

# In[38]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your odometry function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

# TODO: write the odometry function

import numpy as np 

def poseEstimation( R, # radius of wheel (assumed identical) - this is fixed in simulation, and will be imported from your saved calibration for the physical robot
                    baseline_wheel2wheel, # distance from wheel to wheel; 2L of the theory
                    x_prev, # previous x estimate - assume given
                    y_prev, # previous y estimate - assume given
                    theta_prev, # previous orientation estimate - assume given
                    delta_phi_left, # left wheel rotation (rad)
                    delta_phi_right): # right wheel rotation (rad)
    
    """
        Calculate the current Duckiebot pose using the dead-reckoning approach.

        Returns x,y,theta current estimates:
            x_curr, y_curr, theta_curr (:double: values)
    """
    
#     # TODO: these are random values, you have to implement your own solution in here
    dr = R * delta_phi_right
    dl = R * delta_phi_left
    dA = (dr + dl)/2.0 
    d_theta = (dr - dl) / baseline_wheel2wheel
    
    # Question: Why it is using the previous theta???
    x_curr = x_prev + dA * np.cos(theta_prev)  
    y_curr = y_prev + dA * np.sin(theta_prev)
    theta_curr = theta_prev + d_theta
#     print("\t",round(theta_prev,3), round(x_prev,3),round(y_prev,3))
#     print("\t\t",round(delta_phi_left,3), round(delta_phi_right,3))
#     print("\t\t\t",round(dl,3),round(dr,3),round(dA,3),round( d_theta,3),round(x_curr,3),round(y_curr,3))
    
    return x_curr, y_curr, theta_curr
