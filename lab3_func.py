#!/usr/bin/env python

import cv2
import numpy as np 

"""
To init blob search params, will be init (called) in the ImageConverter class
"""
def blob_search_init():

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    ################# Your Code Starts Here #################

    # Filter by Color 
    params.filterByColor = True
    params.blobColor = 255


    # Filter by Area.
    params.filterByArea = False    
    params.minArea = 1500

    # Filter by Circularity
    params.filterByCircularity = False
    params.minCircularity = 0.1

    # Filter by Inerita
    params.filterByInertia = False
    params.minInertiaRatio = 0.87

    # Filter by Convexity
    params.filterByConvexity = False
    params.minConvexity = 0.01

    # Any other params to set???


    ################## Your Code Ends Here ##################

    # Create a detector with the parameters
    blob_detector = cv2.SimpleBlobDetector_create(params)

    return blob_detector


"""
To find blobs in an image, will be called in the callback function of image_sub subscriber
"""
def blob_search(image, detector):

    # Convert the color image into the HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


    ############################ Your Code Starts Here ############################

    # Find lower & upper for orange
    
    # lower = (110,50,50)      # blue lower
    # upper = (130,255,255)   # blue upper
    lower = (8,200,50)      # orange lower
    upper = (15,255,255)   # orange upper

    ############################# Your Code Ends Here #############################


    # Define a mask using the lower and upper bounds of the orange color 
    mask_image = cv2.inRange(hsv_image, lower, upper)

    crop_top_row = 100
    crop_bottom_row = 350
    crop_top_col = 200
    crop_bottom_col = 550

    crop_image = mask_image[crop_top_row:crop_bottom_row, crop_top_col:crop_bottom_col]

    blob_image_center = []

    ############################ Your Code Start Here ############################

    # Call opencv simpleBlobDetector functions here to find centroid of all large enough blobs in 
    # crop_image. Make sure to add crop_top_row and crop_top_col to the centroid row and column found

    # Make sure this blob center is in the full image pixels not the cropped image pixels
    


    # Draw centers on each blob, append all the centers to blob_image_center as string in format "x y"
    keypoints = detector.detect(crop_image)
    x_t = 0
    y_t = 0
    counter = 0
    im_with_keypoints = image
    for keypoint in keypoints:
        x = int(keypoint.pt[0])
        y = int(keypoint.pt[1])
        x_t = x_t+x
        y_t = y_t+y
        counter = counter+1
        im_with_keypoints = cv2.circle(im_with_keypoints, tuple([int(x+crop_top_col), int(y+crop_top_row)]), 3, (0, 0, 255), -1)
        blob_image_center.append(str(x+crop_top_col)+' '+str(y+crop_top_row))

    im_with_keypoints = cv2.circle(im_with_keypoints, tuple([int((x_t/counter)+crop_top_col), int((y_t/counter)+crop_top_row)]), 3, (255, 0, 0), -1)

    #im_with_keypoints =  # Change this to create a proper im_with_keypoints

    ############################# Your Code End Here #############################

    # Draw small circle at pixel coordinate crop_top_col, crop_top_row so you can move a color
    # under that pixel location and see what the HSV values are for that color. 
    pixel = [int(463), int(167)]
    #pixel = [int(crop_top_col), int(crop_top_row)] # Change this to select a new dot location [column, row]
    #im_with_keypoints = cv2.circle(im_with_keypoints, (pixel[0], pixel[1]), 3, (0, 0, 255), -1) #Note: circle uses (c,r) for its center
    print('H,S,V at pixel ' + str(pixel[1]) + ' ' + str(pixel[0]) + ' ' + str(hsv_image[pixel[1],pixel[0]]))    # hsv_image uses [r,c]

    cv2.namedWindow("Maze Window")
    cv2.imshow("Maze Window", im_with_keypoints)

    cv2.namedWindow("MaskImage Window")
    cv2.imshow("MaskImage Window", mask_image)

    cv2.namedWindow("Crop Window")
    cv2.imshow("Crop Window", crop_image)

    cv2.waitKey(2)

    return blob_image_center