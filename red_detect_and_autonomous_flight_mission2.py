#!/usr/bin/env python
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
import time
from pymavlink import mavutil
import cv2
import numpy as np
import os.path

##################################################
vehicle_connect = False
screen = True
draw_lines = True
record = True
field = 100
sualma = 5
redwaypoint = 6
mission_alt = 10
##################################################

gnd_speed = 0.1
pump = True
recordnum = 0
connection = "/dev/ttyACM0"
baudrate = "115200"
complete = False

if vehicle_connect:
    vehicle = connect(connection, baud=baudrate, wait_ready=True)

if pump:
    import RPi.GPIO as GPIO


def pumprecord(pin, delay):
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(pin, GPIO.OUT)

    time.sleep(1)
    print(f"MOTOR ON {pin}")
    GPIO.output(pin, GPIO.HIGH)

    time.sleep(delay)
    print("Motor OFF")
    GPIO.output(pin, GPIO.LOW)


def recordcalc(recordnum):
    while True:
        if os.path.isfile(f"ikincigorev{recordnum}.avi"):
            recordnum += 1
        else:
            return recordnum


def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:  # Trigger just below target alt.
            print("Reached target altitude")
            vehicle.gimbal.rotate(-90, 0, 0)
            break
        time.sleep(1)


def set_velocity_body(vehicle, vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


def condition_yaw(heading, relative=False):
    if relative:
        is_relative = 1  # yaw relative to direction of travel
    else:
        is_relative = 0  # yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        heading,  # param 1, yaw in degrees
        0,  # param 2, yaw speed deg/s
        1,  # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)  # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def goto_position_target_global_int(aLocation, altitude):
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
        0b0000111111111000,  # type_mask (only speeds enabled)
        int(aLocation.lat * 1e7),  # lat_int - X Position in WGS84 frame in 1e7 * meters
        int(aLocation.lon * 1e7),  # lon_int - Y Position in WGS84 frame in 1e7 * meters
        altitude,
        # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0,  # X velocity in NED frame in m/s
        0,  # Y velocity in NED frame in m/s
        0,  # Z velocity in NED frame in m/s
        0, 0, 0,  # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)


def missionstart(connection="/dev/ttyACM0"):
    if vehicle_connect:
        vehicle = connect(connection, baud=baudrate, wait_ready=True)
        arm_and_takeoff(mission_alt)
        vehicle.mode = VehicleMode("AUTO")


def reddetect(screen, record):
    target = None
    record_num = recordcalc(1)
    cap = cv2.VideoCapture(0)
    pTime = 0
    if record:
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        writer = cv2.VideoWriter(f'ikincigorev{record_num}.avi', fourcc, 20.0, (640, 480))

    while True:
        success, frame = cap.read()
        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime
        if vehicle_connect:
            if vehicle.commands.next == sualma:
                pumprecord(26, 20)
        if success:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
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

            if draw_lines:
                cv2.line(frame, (int(w / 3), 0), (int(w / 3), h), color=(0, 255, 0))
                cv2.line(frame, (int((2 * w) / 3), 0), (int((2 * w) / 3), h), color=(0, 255, 0))
                cv2.line(frame, (0, int((2 * h) / 3)), (w, int((2 * h) / 3)), color=(255, 0, 0))
                cv2.line(frame, (0, int(h / 3)), (w, int(h / 3)), color=(255, 0, 0))
                #cv2.line(frame, (0, int((12 * h) / 12)), (w, int((12 * h) / 12)), color=(0, 255, 0), thickness=3)
                #cv2.line(frame, (0, int((4 * h) / 12)), (w, int((4 * h) / 12)), color=(0, 255, 0), thickness=3)
                #cv2.line(frame, (0, int((10 * h) / 12)), (w, int((10 * h) / 12)), color=(0, 255, 0), thickness=3)

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

                    rightbound = int((2 * w) / 3)
                    leftbound = int(w / 3)
                    upperbound = int(h / 3)
                    lowerbound = int((2 * h) / 3)
                    target= vehicle.location.global_relative_frame
                    print("target saved")

                    #if rightbound > center[0] > leftbound:
                    #    if lowerbound > center[1] > upperbound:
                    #	    if vehicle_connect:
                    #            target = vehicle.location.global_relative_frame

                    #            print("hedef kayıt edildi")

            print(vehicle.location.global_relative_frame)
            print(target)
            print(vehicle.commands.next)
            
                
            while vehicle.commands.next == 6 and complete == False:
                
                vehicle.mode = VehicleMode("GUIDED")
                print("guided")
                goto_position_target_global_int(vehicle.location.global_relative_frame, mission_alt)
                time.sleep(4)
                print("mode guided")
                goto_position_target_global_int(target, mission_alt)
                time.sleep(10)
                complete = True
            
                    

                    
            if center[0] > rightbound:
                set_velocity_body(vehicle, 0, gnd_speed, 0)
                time.sleep(2)
            elif center[0] < leftbound:
                set_velocity_body(vehicle, 0, -gnd_speed, 0)
                time.sleep(2)
            elif rightbound > center[0] > leftbound:
                if center[2] > lowerbound:
                    set_velocity_body(vehicle, gnd_speed, 0, 0)
                    time.sleep(2)
                elif center[2] < upperbound:
                    set_velocity_body(vehicle, -gnd_speed, 0, 0)
                    time.sleep(2)
                elif lowerbound > center[1] > upperbound:
                    goto_position_target_global_int(vehicle.location.global_relative_frame, mission_alt-7)
                    pumprecord(27, 25)
                    time.sleep(10)
                    vehicle.mode = VehicleMode("AUTO")
                    break
        if record:
            writer.write(frame)
        if screen:
            cv2.imshow('image', frame)
        cv2.waitKey(1)


reddetect(screen, record)