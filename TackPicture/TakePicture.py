import numpy as np
import cv2

cap = cv2.VideoCapture(4)
frameId = 0
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    cv2.imshow('frame',frame)

    if cv2.waitKey(1) & 0xFF == ord('t'):
        filename = "./pic/camera-pic-of-charucoboard-" +  str(int(frameId)) + ".jpg"
        cv2.imwrite(filename, frame)
        frameId += 1

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
# cv2.waitKey(1) & 
