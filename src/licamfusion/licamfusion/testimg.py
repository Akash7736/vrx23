import cv2
img = cv2.imread('camera_image.jpeg')
for i, row in enumerate(img): 
  
  # get the pixel values by iterating 
    for j, pixel in enumerate(img): 
        if(i == j or i+j == img.shape[0]): 
  
                # update the pixel value to black 
            img[i][j] = [0, 0, 0] 
  
# display image 
print(img.shape)