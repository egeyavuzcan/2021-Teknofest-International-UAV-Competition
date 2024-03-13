#!/usr/bin/env python
import numpy as np
import cv2
import time


cap = cv2.VideoCapture(0)
while True:
    success , frame = cap.read()
    if success:
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        blurred = cv2.GaussianBlur(frame,(11,11),3)
        blurred_hsv = cv2.cvtColor(blurred,cv2.COLOR_BGR2HSV)
        low_red = np.array([155, 105, 80]) #(48,90,60)
        #s min 20-80
        high_red = np.array([179, 255, 255])
        mask = cv2.inRange(blurred_hsv,low_red,high_red)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        orji = cv2.bitwise_and(frame, frame, mask=mask)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        #cv2.imshow("mask",mask)
        #cv2.imshow("orjinal",frame)
        #cv2.imshow("hsv",blurred_hsv)
        (contours, _) = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        Center = None

        if len(contours) > 0:
            #time.sleep(1)
            c = max(contours,key=cv2.contourArea)

            rect = cv2.minAreaRect(c)
            ((x, y), (width, height), rotation) = rect

            s = "x: {}, y: {}, width: {}, height: {}, rotation: {}".format(np.round(x), np.round(y), np.round(width),
                                                                           np.round(height), np.round(rotation))
            print(s)
            kutu = cv2.boxPoints(rect)
            kutu = np.int64(kutu)


            M = cv2.moments(c)
            center2 = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            left = tuple(c[c[:, :, 0].argmin()][0])
            right = tuple(c[c[:, :, 0].argmax()][0])
            top = tuple(c[c[:, :, 1].argmin()][0])
            bottom = tuple(c[c[:, :, 1].argmax()][0])
            cv2.circle(frame, left, 2, (0, 50, 255), -1)
            cv2.circle(frame, right, 2, (0, 255, 255), -1)
            cv2.circle(frame, top, 2, (0, 255, 255), -1)
            sol = int(left[0] + (right[0] - left[0]) / 2) if right[0] > left[0] else int(
                right[0] + (left[0] - right[0]) / 2)
            sag = int(left[1] + (right[1] - left[1]) / 2) if right[1] > left[1] else int(
                right[1] + (left[1] - right[1]) / 2)
            center = tuple((sol, sag))
            cv2.drawContours(frame,[kutu],0,(0,255,255),2)
            cv2.circle(frame,center2,5,(0,255,0),-1)
            cv2.circle(frame,center,5,(255,0,255),-1)
            cv2.putText(frame, s, (25, 50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 255, 255), 2)
                            cv2.drawContours(frame, [box], 0, (0, 255, 255), 2)
            if area 
                rightbound = int( (2 * w) /3)
                leftbound = int(w / 3)
                upperbound = int((2*h)/3)
                lowerbound = int(h/3)
                #vehicle.mode = VehicleMode("GUIDED")
                if leftbound < center[0] < rightbound:
                    #git
                    #print(center[1],(2*h)/3)
                    if (5*h)/12 > center[1] > (4*h)/12:
                        #print("en başta = ", ilkTurdaTanimama)
                        if ilkTurdaTanimama == True:
                            print("gördüm ama inmicem")
                            time.sleep(3)
                            ilkTurdaTanimama = False
                            print("if içinde = ",ilkTurdaTanimama)
                        elif ilkTurdaTanimama == False:
                            print("alanda kırmızı var")
                            """
                            vehicle.mode = VehicleMode("GUIDED")
                            goto_position_target_global_int(vehicle.location.global_relative_frame, 3)
                            time.sleep(20)
                            goto_position_target_global_int(vehicle.location.global_relative_frame, 10)
                            vehicle.mode = VehicleMode("AUTO")
                            #time.sleep(10)
                            """
                            ilkTurdaTanimama = 5
                elif center[0] < leftbound:
                    degree = degree - 5
                    #condition_yaw(degree)
                    print(" sol dönüyoruz")
                    #time.sleep(0.5)
                    pass
                elif center[0] > rightbound:
                    degree += 5
                    #condition_yaw(degree)
                    print("sağ dönüyoruz")
                    #time.sleep(0.5)
                    pass

        cv2.imshow("orjitanl tespit",frame)
        #cv2.imshow("orji",orji)
    else:
        break

    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

cap.release()
cv2.destroyAllWindows()

