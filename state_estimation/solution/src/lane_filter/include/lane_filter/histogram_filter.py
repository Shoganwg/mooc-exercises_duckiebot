#!/usr/bin/env python
# coding: utf-8

# In[14]:


# start by importing some things we will need
import cv2
import matplotlib
import numpy as np
from scipy.ndimage.filters import gaussian_filter
from scipy.stats import entropy, multivariate_normal
from math import floor, sqrt

# In[19]:


# Now let's define the prior function. In this case we choose
# to initialize the historgram based on a Gaussian distribution around [0,0]
def histogram_prior(belief, grid_spec, mean_0, cov_0):
    pos = np.empty(belief.shape + (2,))
    pos[:, :, 0] = grid_spec["d"]
    pos[:, :, 1] = grid_spec["phi"]
    RV = multivariate_normal(mean_0, cov_0)
    belief = RV.pdf(pos)
    return belief

# In[191]:


# Now let's define the predict function

def histogram_predict(belief, dt, left_encoder_ticks, right_encoder_ticks, grid_spec, robot_spec, cov_mask):
        belief_in = belief
        delta_t = dt
        
        # TODO calculate v and w from ticks using kinematics. You will need  some parameters in the `robot_spec` defined above
        rad_per_tk = 2 * np.pi / robot_spec["encoder_resolution"]
        r = robot_spec["wheel_radius"]
        vl, vr = (left_encoder_ticks * rad_per_tk * r)/ dt , (right_encoder_ticks * rad_per_tk * r)/ dt
        # for ele in [vl,left_encoder_ticks,rad_per_tk]:
            # print(f" {namestr(ele, globals())}  = {ele} \n\n")
        L = robot_spec["wheel_baseline"] / 2
        v = (vl + vr)/2 # replace this with a function that uses the encoder 
        # print("v = ",v)
        w =  (vr-vl)/(2*L)# replace this with a function that uses the encoder
        
        # TODO propagate each centroid forward using the kinematic function
        d_t = grid_spec['d'] + v * dt * np.sin(grid_spec['phi'])# replace this with something that adds the new odometry
        phi_t = grid_spec['phi'] + w * dt # replace this with something that adds the new odometry

        p_belief = np.zeros(belief.shape)

        # Accumulate the mass for each cell as a result of the propagation step
        for i in range(belief.shape[0]):
            for j in range(belief.shape[1]):
                # If belief[i,j] there was no mass to move in the first place
                if belief[i, j] > 0:
                    # Now check that the centroid of the cell wasn't propagated out of the allowable range
                    if (
                        d_t[i, j] > grid_spec['d_max']
                        or d_t[i, j] < grid_spec['d_min']
                        or phi_t[i, j] < grid_spec['phi_min']
                        or phi_t[i, j] > grid_spec['phi_max']
                    ):
                        continue
                    
                    # TODO Now find the cell where the new mass should be added
                    gd = grid_spec["delta_d"]
                    moved = v * dt * np.sin(grid_spec['phi'][i,j]) 
                    moved_sign = np.sign(moved)
                    dcmd = abs(moved + (gd/2) * moved_sign)/gd
                    i_new = i + int(moved_sign * int(dcmd)) # replace with something that accounts for the movement of the robot
                    
                    gphi = grid_spec["delta_phi"]
                    movep = w * dt 
                    movep_sign = np.sign(movep)
                    dcmp =  abs(movep + (gphi/2) * movep_sign)/gphi
                    j_new = j + int(movep_sign * int(dcmp))# replace with something that accounts for the movement of the robot
                    
                    if (i_new <0 or i_new >=belief.shape[0]) or (j_new <0 or j_new >=belief.shape[1]):
                        continue
                    
                    p_belief[i_new, j_new] += belief[i, j]

        # Finally we are going to add some "noise" according to the process model noise
        # This is implemented as a Gaussian blur
        s_belief = np.zeros(belief.shape)
        gaussian_filter(p_belief, cov_mask, output=s_belief, mode="constant")

        if np.sum(s_belief) == 0:
            return belief_in
        belief = s_belief / np.sum(s_belief)
        return belief

    
# {'mean_d_0': 0, 'mean_phi_0': 0, 'sigma_d_0': 0.1, 'sigma_phi_0': 0.1, 
#'delta_d': 0.02, 'delta_phi': 0.1, 'd_max': 0.3, 'd_min': -0.15, 'phi_min': -1.5, 
# 'phi_max': 1.5, 'linewidth_white': 0.05, 'linewidth_yellow': 0.025, 'lanewidth': 0.23, 
# 'sigma_d_mask': 1.0, 'sigma_phi_mask': 2.0, 'encoder_resolution': 135, 'wheel_radius': 0.0318, 'wheel_baseline': 0.1}
# {'linewidth_white': 0.05, 'linewidth_yellow': 0.025, 'lanewidth': 0.23}
# {'wheel_radius': 0.0318, 'wheel_baseline': 0.1, 'encoder_resolution': 135}
# [1.0, 2.0]   delta_d 0.02   delta_phi 0.1  d_min -0.15  d_max 0.3  phi_min -1.5   phi_max 1.5

# In[192]:


# We will start by doing a little bit of processing on the segments to remove anything that is behing the robot (why would it be behind?)
# or a color not equal to yellow or white

def prepare_segments(segments):
    filtered_segments = []
    for segment in segments:

        # we don't care about RED ones for now
        if segment.color != segment.WHITE and segment.color != segment.YELLOW:
            continue
        # filter out any segments that are behind us
        if segment.points[0].x < 0 or segment.points[1].x < 0:
            continue

        filtered_segments.append(segment)
    return filtered_segments

# In[193]:



def generate_vote(segment, road_spec):
    p1 = np.array([segment.points[0].x, segment.points[0].y])
    p2 = np.array([segment.points[1].x, segment.points[1].y])
    t_hat = (p2 - p1) / np.linalg.norm(p2 - p1)
    n_hat = np.array([-t_hat[1], t_hat[0]])
    
    d1 = np.inner(n_hat, p1)
    d2 = np.inner(n_hat, p2)
    l1 = np.inner(t_hat, p1)
    l2 = np.inner(t_hat, p2)
    if l1 < 0:
        l1 = -l1
    if l2 < 0:
        l2 = -l2

    l_i = (l1 + l2) / 2
    d_i = (d1 + d2) / 2
    phi_i = np.arcsin(t_hat[1])
    if segment.color == segment.WHITE:  # right lane is white
        if p1[0] > p2[0]:  # right edge of white lane
            d_i -= road_spec['linewidth_white']
        else:  # left edge of white lane
            d_i = -d_i
            phi_i = -phi_i
        d_i -= road_spec['lanewidth'] / 2

    elif segment.color == segment.YELLOW:  # left lane is yellow
        if p2[0] > p1[0]:  # left edge of yellow lane
            d_i -= road_spec['linewidth_yellow']
            phi_i = -phi_i
        else:  # right edge of white lane
            d_i = -d_i
        d_i = road_spec['lanewidth'] / 2 - d_i

    return d_i, phi_i

# In[194]:


def generate_measurement_likelihood(segments, road_spec, grid_spec):

    # initialize measurement likelihood to all zeros
    measurement_likelihood = np.zeros(grid_spec['d'].shape)

    for segment in segments:
        d_i, phi_i = generate_vote(segment, road_spec)

        # if the vote lands outside of the histogram discard it
        if d_i > grid_spec['d_max'] or d_i < grid_spec['d_min'] or phi_i < grid_spec['phi_min'] or phi_i > grid_spec['phi_max']:
            continue

        # TODO find the cell index that corresponds to the measurement d_i, phi_i
        gd = grid_spec["delta_d"]
        dmin = grid_spec["d_min"]
        absdmin = abs(dmin)
        i = int((d_i + absdmin)/gd)
        
        gphi = grid_spec["delta_phi"]
        phimin = grid_spec["phi_min"]
        absphimin = abs(phimin)
        j = int((phi_i+absphimin) / gphi)
        
        # i = 1 # replace this
        # j = 1 # replace this
        
        # Add one vote to that cell
        measurement_likelihood[i, j] += 1

    if np.linalg.norm(measurement_likelihood) == 0:
        return None
    measurement_likelihood /= np.sum(measurement_likelihood)
    return measurement_likelihood


# In[195]:


def histogram_update(belief, segments, road_spec, grid_spec):
    # prepare the segments for each belief array
    segmentsArray = prepare_segments(segments)
    # generate all belief arrays

    measurement_likelihood = generate_measurement_likelihood(segmentsArray, road_spec, grid_spec)

    if measurement_likelihood is not None:
        # TODO: combine the prior belief and the measurement likelihood to get the posterior belief
        # Don't forget that you may need to normalize to ensure that the output is valid probability distribution
        # belief = measurement_likelihood # replace this with something that combines the belief and the measurement_likelihood
        belief = belief * measurement_likelihood
        if np.sum(belief) == 0:
            return belief_in
        belief = belief / np.sum(belief)

    return (measurement_likelihood, belief)

