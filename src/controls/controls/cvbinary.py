import cv2
import numpy as np

def on_threshold1_change(value):
    global img, win_name
    _, binary_image = cv2.threshold(img, value, threshold2_value, cv2.THRESH_BINARY)
    cv2.imshow(win_name, binary_image)

def on_threshold2_change(value):
    global img, win_name
    _, binary_image = cv2.threshold(img, threshold1_value, value, cv2.THRESH_BINARY)
    cv2.imshow(win_name, binary_image)

# Load image
img = cv2.imread('/home/guildstudent/akash_ws/src/controls/controls/IMLdataset/dataset_28_01_2024_13_07_35/front_left_imgs/frontL_4000000.png', cv2.IMREAD_GRAYSCALE)

# Check if the image is loaded successfully
if img is None:
    print("Error: Could not load the image.")
else:
    # Create a window
    win_name = 'Binary Image'
    cv2.namedWindow(win_name)

    # Initial threshold values
    threshold1_value = 127
    threshold2_value = 255

    # Create sliders
    cv2.createTrackbar('Threshold 1', win_name, threshold1_value, 255, on_threshold1_change)
    cv2.createTrackbar('Threshold 2', win_name, threshold2_value, 255, on_threshold2_change)

    # Display initial binary image
    _, binary_image = cv2.threshold(img, threshold1_value, threshold2_value, cv2.THRESH_BINARY)
    cv2.imshow(win_name, binary_image)

    # Wait for user input
    cv2.waitKey(0)

    # Close all windows
    cv2.destroyAllWindows()
