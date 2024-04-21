from PIL import Image 
import cv2
  
# img = Image.open("west.jpg") 

# img = img.rotate(-90) 
left = 3700
top =  4000 
right = 3700+440
bottom =  4000 + 900
 
  
# img_res = img.crop((left, top, right, bottom)) 


img = cv2.imread('20230611_112831.jpg')
# img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)

x,y,w,h = [3310, 3685, 2540,4570]
img = img[y-(h//2):y+(h//2), x-(w//2):x+(w//2)]

# img2 =cv2.resize(img, (64,64))
cv2.imshow('redangle',img)
cv2.imwrite('cropped2.jpg',img)
cv2.destroyAllWindows()
cv2.waitKey(0)


# img_res.show() 