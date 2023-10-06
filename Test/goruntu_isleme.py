import cv2
import numpy as np

cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    if(cap.isOpened() == False):
        continue
    else:
        blur = cv2.medianBlur(frame, 51)
        hsv = cv2.cvtColor(blur, cv2.COLOR_RGB2HSV) 
        lower_red = np.array([0,70,50])
        upper_red = np.array([10,255,255])
        mask = cv2.inRange(hsv, lower_red, upper_red) 

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

        for cnt in contours:
            area = cv2.contourArea(cnt)

            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)

            x = approx.ravel()[0]
            y = approx.ravel()[1]

            if area > 10000:
                cv2.drawContours(frame, [approx], 0, (0, 0, 255), 5)

                if len(approx) > 6:
                    M = cv2.moments(mask)

                    X = int(M["m10"] / M["m00"])
                    Y = int(M["m01"] / M["m00"])
                    print(X, Y)

                    cv2.circle(mask, (X, Y), 5, (0, 255, 0), -1)

        cv2.imshow("Ana_Goruntu", frame)
        cv2.imshow("Mask", mask)

        if cv2.waitKey(60) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()

