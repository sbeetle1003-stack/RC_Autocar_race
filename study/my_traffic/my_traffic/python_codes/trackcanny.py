import cv2
import numpy as np

img = cv2.imread('traffic_light.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
blur_gray = cv2.GaussianBlur(gray,(5, 5), 0)

def nothing(x):
    pass

cv2.namedWindow('canny')
cv2.createTrackbar('Threshold1','canny',1,200, nothing)
cv2.createTrackbar('Threshold2','canny',1,255, nothing)

while True:
    if cv2.waitKey(1) & 0xFF == 27:
        break
    th1 = cv2.getTrackbarPos('Threshold1','canny')
    th2 = cv2.getTrackbarPos('Threshold2','canny')

    if th1 == 0:
        th1 = 1
    if th2 == 0:
        th2 = 1

    edge_img = cv2.Canny(np.uint8(blur_gray), th1, th2)
    cv2.imshow('canny',edge_img)

cv2.destroyAllWindows()
