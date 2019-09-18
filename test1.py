import cv2

img = cv2.imread("D:\\Study\\Nam3\\intern\\stereo_camera.jpg")
size = img.shape
new_img = cv2.resize(img,(size[0],size[1]))
cv2.imwrite("D:\\Study\\Nam3\\intern\\new_stereo_camera.jpg")