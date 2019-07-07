from __future__ import print_function
import numpy as np 
import cv2
from scipy.spatial import distance as dist 
import imutils
from imutils import contours

low_thresh = 0
high_thresh = 255
window_detection_name = 'Object Detection'
low_thresh_name = 'Low thresh'
high_thresh_name = 'High thresh'



def on_low_thresh_trackbar(val):
    global low_thresh
    global high_thresh
    low_thresh = val
    low_thresh = min(high_thresh-1, low_thresh)
    cv2.setTrackbarPos(low_thresh_name, window_detection_name, low_thresh)


def on_high_thresh_trackbar(val):
    global low_thresh
    global high_thresh
    high_thresh = val
    high_thresh = max(high_thresh, low_thresh+1)
    cv2.setTrackbarPos(high_thresh_name, window_detection_name, high_thresh)





cv2.namedWindow(window_detection_name)
cv2.createTrackbar(low_thresh_name, window_detection_name, low_thresh,
                   255, on_low_thresh_trackbar)
cv2.createTrackbar(high_thresh_name, window_detection_name, high_thresh,
                   255, on_high_thresh_trackbar)






def order_points(pts):
    xSorted = pts[np.argsort(pts[:,0]),:]

    #take the fisrt 2 left most and the last 2 right most
    leftMost = xSorted[:2,:]
    rightMost = xSorted[:2,:]

    #with the 2 left most, we can find which one is top or bottom by sort
    #it by y-coordinate
    leftMost = leftMost[np.argsort(leftMost[:,1]),:]
    (tl,bl) = leftMost #tl is top-left and bl is bottom-left

    #do the same with top-right and bottom right
    rightMost = rightMost[np.argsort(rightMost[:,1]),:]
    (tr,br) = rightMost #tr is top-right and br is bottom-right

    return np.array([tl,tr,br,bl],dtype='float32')

#test
cap = cv2.VideoCapture(0)

while True:
    ret,frame = cap.read()
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(frame,(7,7),0)

    edged = cv2.Canny(gray,low_thresh,high_thresh)
    edged = cv2.dilate(edged,None,iterations=1)
    edged = cv2.erode(edged,None,iterations=1)

    #find contours in edged map
    ctns = cv2.findContours(edged.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    ctns = imutils.grab_contours(ctns)

    # sort the contours from left-to-right and initialize the bounding box
    # point colors
    (ctns,_) = contours.sort_contours(ctns)
    colors = ((0, 0, 255), (240, 0, 159), (255, 0, 0), (255, 255, 0))

    for (i,c) in enumerate(ctns):
        if cv2.contourArea(c) < 10:
            continue
        
        box = cv2.minAreaRect(c)
        box = cv2.boxPoints(box)
        box = np.array(box,dtype='int')
        cv2.drawContours(frame,[box],-1,(0,255,0),2)
        # show the original coordinates
        print("Object #{}:".format(i + 1))
        print(box)

        rect = order_points(box)
        print(rect.astype("int"))
        print("")

        for ((x, y), color) in zip(rect, colors):
            cv2.circle(frame, (int(x), int(y)), 5, color, -1)
        cv2.putText(frame, "Object #{}".format(i + 1),
		(int(rect[0][0] - 15), int(rect[0][1] - 15)),
		cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
 
	# show the image
    cv2.imshow('original image',frame)
    cv2.imshow('canny',edged)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()