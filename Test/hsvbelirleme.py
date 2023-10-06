import cv2
import numpy as np

def callback(x):
	pass

cap = cv2.imread('/mnt/d/Desktop/havuz.png')
cv2.namedWindow('image')

ilowH = 92
ihighH = 106

ilowS = 72
ihighS = 255
ilowV = 30
ihighV = 255

# create trackbars for color change
cv2.createTrackbar('lowH','image',ilowH,179,callback)
cv2.createTrackbar('highH','image',ihighH,179,callback)

cv2.createTrackbar('lowS','image',ilowS,255,callback)
cv2.createTrackbar('highS','image',ihighS,255,callback)

cv2.createTrackbar('lowV','image',ilowV,255,callback)
cv2.createTrackbar('highV','image',ihighV,255,callback)

flag = 0

while True:
    # grab the frame

    # get trackbar positions
	cap = cv2.VideoCapture('/mnt/d/Desktop/havuz.png')
	ret, frame = cap.read()

	ilowH = cv2.getTrackbarPos('lowH', 'image')
	ihighH = cv2.getTrackbarPos('highH', 'image')
	ilowS = cv2.getTrackbarPos('lowS', 'image')
	ihighS = cv2.getTrackbarPos('highS', 'image')
	ilowV = cv2.getTrackbarPos('lowV', 'image')
	ihighV = cv2.getTrackbarPos('highV', 'image')
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	lower_hsv = np.array([ilowH, ilowS, ilowV])
	higher_hsv = np.array([ihighH, ihighS, ihighV])
	mask = cv2.inRange(hsv, lower_hsv, higher_hsv)

	frame = cv2.bitwise_and(frame, frame, mask=mask)
	cv2.imshow('image', frame)
	if flag == 0:
		flag = 1

    # show thresholded image
	k = cv2.waitKey(1000) & 0xFF # large wait time to remove freezing
	if k == 113 or k == 27:
	    break


cv2.destroyAllWindows()
cap.release()