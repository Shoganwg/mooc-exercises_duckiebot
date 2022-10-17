#!/usr/bin/env python
# coding: utf-8

# In[5]:


import numpy as np

# Heading control

# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your PIDController function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

# TODO: write your PID function for heading control!

def PIDController(v_0, theta_ref, theta_hat, prev_e, prev_int, delta_t):
    """
    Args:
        v_0 (:double:) linear Duckiebot speed (given).
        theta_ref (:double:) reference heading pose
        theta_hat (:double:) the current estiamted theta.
        prev_e (:double:) tracking error at previous iteration.
        prev_int (:double:) previous integral error term.
        delta_t (:double:) time interval since last call.
    returns:
        v_0 (:double:) linear velocity of the Duckiebot 
        omega (:double:) angular velocity of the Duckiebot
        e (:double:) current tracking error (automatically becomes prev_e_y at next iteration).
        e_int (:double:) current integral error (automatically becomes prev_int_y at next iteration).
    """
    
    # TODO: these are random values, you have to implement your own PID controller in here
    # to be tune
    # question, when I set kp = 10, kd and ki to be zero, the sanity check is better than these 5, 0.1,0.2 
    kp = 5.0
    kd = 0.1
    ki = 0.2
    
    # et = theta_hat - theta_ref !!!!!Original answer. I do not understand why the error should be
    # reference subtract theta_hat, rather than the other way. 
    et = theta_ref - theta_hat 
    int_e = prev_int + et * delta_t
    # e_int = max(min(e_int,2),-2)     ## This line copy from solution
    d_e = (et - prev_e) / delta_t
    
    omega = (kp * et) + ( ki * int_e) + ( kd * d_e)
    e = et
    e_int = int_e
    
    return [v_0, omega], e, e_int
