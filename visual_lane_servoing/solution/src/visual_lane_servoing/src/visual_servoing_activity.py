#!/usr/bin/env python
# coding: utf-8

# In[65]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

import cv2
import numpy as np


def get_steer_matrix_left_lane_markings(shape):
    """
        Args:
            shape: The shape of the steer matrix (tuple of ints)
        Return:
            steer_matrix_left_lane: The steering (angular rate) matrix for Braitenberg-like control 
                                    using the masked left lane markings (numpy.ndarray)
    """
    
    # steer_matrix_left_lane = np.random.rand(*shape)
    # steer_matrix_left_lane = np.zeros(shape, dtype=np.uint8)
    steer_matrix_left_lane = np.zeros(shape)
    # rown = int(shape[0] * 2/5)
    rown = 0
    coln = int(shape[1] * 1/2)
    steer_matrix_left_lane[rown:,0:coln] = -1
    # steer_matrix_left_lane[rown:,coln:] = 1

    return steer_matrix_left_lane

# In[67]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK


def get_steer_matrix_right_lane_markings(shape):
    """
        Args:
            shape: The shape of the steer matrix (tuple of ints)
        Return:
            steer_matrix_right_lane: The steering (angular rate) matrix for Braitenberg-like control 
                                     using the masked right lane markings (numpy.ndarray)
    """
    
    # steer_matrix_right_lane = np.random.rand(*shape)
    # steer_matrix_right_lane = np.zeros(shape, dtype=np.uint8)
    steer_matrix_right_lane = np.zeros(shape)
    # rown = int(shape[0] * 2/5)
    rown = 0
    coln = int(shape[1] * 1/2)
    steer_matrix_right_lane[rown:,coln:] = 1
    # steer_matrix_right_lane[rown:,:coln] = -1

    return steer_matrix_right_lane

print(get_steer_matrix_left_lane_markings((6,6)))
print(get_steer_matrix_right_lane_markings((6,6)) )

# In[63]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

import cv2
import numpy as np


def detect_lane_markings(image):
    """
        Args:
            image: An image from the robot's camera in the BGR color space (numpy.ndarray)
        Return:
            left_masked_img:   Masked image for the dashed-yellow line (numpy.ndarray)
            right_masked_img:  Masked image for the solid-white line (numpy.ndarray)
    """
    
    h, w, _ = image.shape
    
    imgbgr = image
    
    # Convert the image to HSV for any color-based filtering
    imghsv = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2HSV)

    # Most of our operations will be performed on the grayscale version
    img = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2GRAY)
    
    sigma = 4 # CHANGE ME

    # Smooth the image using a Gaussian kernel
    img_gaussian_filter = cv2.GaussianBlur(img,(0,0), sigma)
    img  = img_gaussian_filter
    
    # Convolve the image with the Sobel operator (filter) to compute the numerical derivatives in the x and y directions
    sobelx = cv2.Sobel(img,cv2.CV_64F,1,0)
    sobely = cv2.Sobel(img,cv2.CV_64F,0,1)

    # Compute the magnitude of the gradients
    Gmag = np.sqrt(sobelx*sobelx + sobely*sobely)

    # Compute the orientation of the gradients
    Gdir = cv2.phase(np.array(sobelx, np.float32), np.array(sobely, dtype=np.float32), angleInDegrees=True)
    
    threshold =  10 # CHANGE ME

    mask_mag = (Gmag > threshold)
    
    # modified and take reference from https://stackoverflow.com/questions/22588146/tracking-white-color-using-python-opencv
    sensitivity = 80
    white_lower_hsv = np.array([0, 0, 255-sensitivity])         # CHANGE ME
    white_upper_hsv = np.array([179, sensitivity, 255])   # CHANGE ME
    # modified and take reference from https://stackoverflow.com/questions/9179189/detect-yellow-color-in-opencv
    yellow_lower_hsv = np.array([20, 100, 100])        # CHANGE ME
    yellow_upper_hsv = np.array([30, 255, 255])  # CHANGE ME

    mask_white = cv2.inRange(imghsv, white_lower_hsv, white_upper_hsv)
    mask_yellow = cv2.inRange(imghsv, yellow_lower_hsv, yellow_upper_hsv)
    
    # Let's create masks for the left- and right-halves of the image
    width = img.shape[1]
    mask_left = np.ones(sobelx.shape)
    mask_left[:,int(np.floor(width/2)):width + 1] = 0
    mask_right = np.ones(sobelx.shape)
    mask_right[:,0:int(np.floor(width/2))] = 0
    
    # In the left-half image, we are interested in the right-half of the dashed yellow line, which corresponds to negative x- and y-derivatives
    # In the right-half image, we are interested in the left-half of the solid white line, which correspons to a positive x-derivative and a negative y-derivative
    # Generate a mask that identifies pixels based on the sign of their x-derivative
    mask_sobelx_pos = (sobelx > 0)
    mask_sobelx_neg = (sobelx < 0)
    mask_sobely_pos = (sobely > 0)
    mask_sobely_neg = (sobely < 0)
    
    # Let's combine these masks with the gradient magnitude mask
    mask_left_edge = mask_left * mask_mag * mask_sobelx_neg * mask_sobely_neg * mask_yellow # * mask_ground # * 
    mask_right_edge = mask_right * mask_mag * mask_sobelx_pos * mask_sobely_neg * mask_white # * mask_ground # * 
    
    mask_left_edge = Gmag * mask_left_edge
    mask_right_edge = Gmag * mask_right_edge
    # mask_left_edge = np.random.rand(h, w)
    # mask_right_edge = np.random.rand(h, w)
    
    return (mask_left_edge, mask_right_edge)
