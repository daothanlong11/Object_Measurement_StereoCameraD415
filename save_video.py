from __future__ import print_function
import numpy as np 
import cv2
 
def save_webcam(outPath,fps,mirror=False):
    # Capturing video from webcam:
    cap = cv2.VideoCapture(0)
 
    currentFrame = 0
 
    # Get current width of frame
    cap.set(3,1280)  # float
    # Get current height of frame
    cap.set(4,720)  # float
 
    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    out = cv2.VideoWriter(outPath, fourcc, fps, (int(1280), int(720)))
 
    while (cap.isOpened()):
 
        # Capture frame-by-frame
        ret, frame = cap.read()
 
        if ret == True:
            if mirror == True:
                # Mirror the output video frame
                frame = cv2.flip(frame, 1)
            # Saves for video
            out.write(frame)
 
            # Display the resulting frame
            cv2.imshow('frame', frame)
        else:
            break
 
        if cv2.waitKey(1) & 0xFF == ord('q'):  # if 'q' is pressed then quit
            break
 
        # To stop duplicate images
        currentFrame += 1
 
    # When everything done, release the capture
    cap.release()
    out.release()
    cv2.destroyAllWindows()
 
def main():
    save_webcam('/home/l-ubuntus/Videos/datasets/video/long.avi', 30.0,mirror=True)
 
if __name__ == '__main__':
    main()

