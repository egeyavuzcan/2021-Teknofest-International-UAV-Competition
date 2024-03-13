import time
import cv2
import numpy as np
import os.path

##################################################
vehicle_connect = False
screen = True
record = False
field = 1000
recordnum = 0

# airspeed = 20
##################################################

def recordcalc(recordnum):
	while True:
		if os.path.isfile(f"ilkgorev{recordnum}.avi"):
			recordnum += 1
		else:
			return recordnum


def reddetect(screen, record):
	record_num = recordcalc(1)
	cap = cv2.VideoCapture(0)
	pTime = 0
	if record:
		fourcc = cv2.VideoWriter_fourcc(*'XVID')
		writer = cv2.VideoWriter(f'ilkgorev{record_num}.avi', fourcc, 20.0, (640, 480))

	while True:
		success, frame = cap.read()
		cTime = time.time()
		fps = 1 / (cTime - pTime)
		pTime = cTime

		if success:
			cv2.putText(frame, f"FPS: {int(fps)}", (20, 70), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255))
			kernelOpen = np.ones((5, 5))
			kernelClose = np.ones((20, 20))
			frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			low_red = np.array([155, 105, 80])
			high_red = np.array([180, 255, 255])
			mask = cv2.inRange(frame_hsv, low_red, high_red)
			maskOpen = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel=kernelOpen)
			maskClose = cv2.morphologyEx(maskOpen, cv2.MORPH_CLOSE, kernelClose)
			mask = maskClose
			(contours, _) = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			h, w = frame.shape[0], frame.shape[1]

			if len(contours) > 0:
				c = max(contours, key=cv2.contourArea)
				rect = cv2.minAreaRect(c)
				box = cv2.boxPoints(rect)
				box = np.int64(box)
				((x, y), (genislik, uzunluk), rotasyon) = rect
				s = "x: {}, y: {}, width: {}, height: {}, rotation: {}".format(np.round(x), np.round(y),
																			   np.round(genislik), np.round(uzunluk),
																			   np.round(rotasyon))
				M = cv2.moments(c)
				center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
				cv2.drawContours(frame, [box], 0, (0, 255, 255), 2)
				cv2.drawContours(frame, contours, 0, (255, 0, 0), 2)
				cv2.circle(frame, center, 5, (255, 0, 255), -1)
				area = cv2.contourArea(c)

				if area > field:
					cv2.drawContours(frame, [box], 0, (0, 255, 255), 2)

		if record:
			writer.write(frame)
		if screen:
			cv2.imshow('image', frame)
		cv2.waitKey(1)


reddetect(screen, record)
