from __future__ import print_function
import cv2
import imutils
from imutils import contours
from imutils import perspective
from scipy.spatial import distance as dist 
import numpy as np 
######################### Initialize Some Stuff ################################

# Define the upper and lower boundaries for a color to be considered "Blue"
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



def midpoint(ptA, ptB):
	return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)

cap = cv2.VideoCapture(0)

while True:
    ret,frame = cap.read()
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame_threshold = cv2.inRange(
        hsv, (low_H, low_S, low_V), (high_H, high_S, high_V))

    res = cv2.bitwise_and(frame, frame, mask=frame_threshold)
    """
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (7, 7), 0)
    edged = cv2.Canny(gray,low_thresh,high_thresh)
    edged = cv2.dilate(edged,None,iterations=1)
    edged = cv2.erode(edged,None,iterations=1)
    cv2.imshow('Edged',edged)
    """
    #find contours in edge map
    ctns = cv2.findContours(edged.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    ctns = imutils.grab_contours(ctns)
    ctns,_ = contours.sort_contours(ctns)
    pixelsPerMetric = None

    for c in ctns:
        orig = frame.copy()
        box = cv2.minAreaRect(c)
        box = cv2.boxPoints(box)
        box = np.array(box,dtype='int')

        box = perspective.order_points(box)
        cv2.drawContours(orig,[box.astype('int')],-1,(123,123,123),2)

        for (x,y) in box:
            cv2.circle(orig,(int(x),int(y)),5,(45,67,89),-1)
        (tl, tr, br, bl) = box
        (tltrX, tltrY) = midpoint(tl, tr)
        (blbrX, blbrY) = midpoint(bl, br)
    
        # compute the midpoint between the top-left and top-right points,
        # followed by the midpoint between the top-righ and bottom-right
        (tlblX, tlblY) = midpoint(tl, bl)
        (trbrX, trbrY) = midpoint(tr, br)
    
        # draw the midpoints on the image
        cv2.circle(orig, (int(tltrX), int(tltrY)), 5, (255, 0, 0), -1)
        cv2.circle(orig, (int(blbrX), int(blbrY)), 5, (255, 0, 0), -1)
        cv2.circle(orig, (int(tlblX), int(tlblY)), 5, (255, 0, 0), -1)
        cv2.circle(orig, (int(trbrX), int(trbrY)), 5, (255, 0, 0), -1)
    
        # draw lines between the midpoints
        cv2.line(orig, (int(tltrX), int(tltrY)), (int(blbrX), int(blbrY)),
            (255, 0, 255), 2)
        cv2.line(orig, (int(tlblX), int(tlblY)), (int(trbrX), int(trbrY)),
            (255, 0, 255), 2)
        # compute the Euclidean distance between the midpoints
        dA = dist.euclidean((tltrX, tltrY), (blbrX, blbrY))
        dB = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))
    
        # if the pixels per metric has not been initialized, then
        # compute it as the ratio of pixels to supplied metric
        # (in this case, inches)
        if pixelsPerMetric is None:
            pixelsPerMetric = dB / 0.955
        # compute the size of the object
        dimA = dA / pixelsPerMetric
        dimB = dB / pixelsPerMetric
    
        # draw the object sizes on the image
        cv2.putText(orig, "{:.1f}in".format(dimA),
            (int(tltrX - 15), int(tltrY - 10)), cv2.FONT_HERSHEY_SIMPLEX,
            0.65, (255, 255, 255), 2)
        cv2.putText(orig, "{:.1f}in".format(dimB),
            (int(trbrX + 10), int(trbrY)), cv2.FONT_HERSHEY_SIMPLEX,
            0.65, (255, 255, 255), 2)
    
        # show the output image
        cv2.imshow("Image", orig)
        cv2.imshow('Gray',gray)
        




    """
    cv2.imshow('gray',gray)
    cv2.imshow('frame', frame)
    #cv2.imshow(window_detection_name, frame_threshold)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break




# Cleanup code
cap.release()
cv2.destroyAllWindows()

