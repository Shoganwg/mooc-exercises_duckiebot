#!/usr/bin/env python
# coding: utf-8

# In[7]:


import numpy as np

# Lateral control

# TODO: write the PID controller using what you've learned in the previous activities

# Note: y_hat will be calculated based on your DeltaPhi() and poseEstimate() functions written previously 

def PIDController(
    v_0, # assume given (by the scenario)
    y_ref, # assume given (by the scenario)
    y_hat, # assume given (by the odometry)
    prev_e_y, # assume given (by the previous iteration of this function)
    prev_int_y, # assume given (by the previous iteration of this function)
    delta_t): # assume given (by the simulator)
    """
    Args:
        v_0 (:double:) linear Duckiebot speed.
        y_ref (:double:) reference lateral pose
        y_hat (:double:) the current estiamted pose along y.
        prev_e_y (:double:) tracking error at previous iteration.
        prev_int_y (:double:) previous integral error term.
        delta_t (:double:) time interval since last call.
    returns:
        v_0 (:double:) linear velocity of the Duckiebot 
        omega (:double:) angular velocity of the Duckiebot
        e_y (:double:) current tracking error (automatically becomes prev_e_y at next iteration).
        e_int_y (:double:) current integral error (automatically becomes prev_int_y at next iteration).
    """
    
    # TODO: these are random values, you have to implement your own PID controller in here
    # to be tune
    
    # et = theta_hat - theta_ref !!!!!Original answer. I do not understand why the error should be
    # reference subtract theta_hat, rather than the other way. 
    et = y_ref - y_hat 
    int_e = prev_int_y + et * delta_t
    int_e = max(min(int_e,2),-2)     ## This line copy from solution
    d_e = (et - prev_e_y) / delta_t
    # 0.1, 7, 0.02  seems to slow in response.  0.1, 8, 0.01 slow in response.
    # 15, 10, 0.01 circuliung
    # kp,kd,ki = 0.1, 2, 0.0
    # 0.1, 4, 0.0
    # 0.8, 5, 0.05
    # 0.2, 5, 0.02
    # best so far 10, 80, 0.01
    kp,kd,ki = 1, 1, 0
    
    omega_y = (kp * et) + ( ki * int_e) + ( kd * d_e)
    e = et
    e_int = int_e
    omega = omega_y
    e_y = et
    e_int_y = e_int  
    return [v_0, omega], e_y, e_int_y

# R,baseline,gain,trim,v_0,y_ref = 0.0318,0.1,0.6,0.0,0.2,0.2
# # unit test input R, baseline, v_0, gain, trim, PIDController
# unit_test = UnitTestPositionPID(R, baseline, v_0, y_ref, gain, trim, PIDController) 
# unit_test.test()
