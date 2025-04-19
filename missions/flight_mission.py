"""
Defines base class for autonomous flight missions.
"""
from abc import ABC, abstractmethod
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import cv2
import numpy as np


class FlightMission(ABC):
    """
    Base class for drone autonomous flight missions.
    """

    def __init__(self, connection_str: str, baud: int = 115200, gnd_speed: float = 0.1):
        """
        Connects to the vehicle and sets parameters.

        Args:
            connection_str (str): MAVLink connection string.
            baud (int): Baudrate for serial.
            gnd_speed (float): Default ground speed.
        """
        self.vehicle = connect(connection_str, baud=baud, wait_ready=True)
        self.gnd_speed = gnd_speed

    def arm_and_takeoff(self, altitude: float):
        """
        Arms the vehicle and takes off to a specified altitude.
        """
        # pre-arm checks omitted for brevity
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            time.sleep(1)
        self.vehicle.simple_takeoff(altitude)
        while True:
            if self.vehicle.location.global_relative_frame.alt >= altitude * 0.95:
                break
            time.sleep(1)

    def goto(self, lat: float, lon: float, alt: float):
        """
        Fly to a GPS coordinate.
        """
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            int(lat * 1e7), int(lon * 1e7), alt,
            0, 0, 0, 0, 0, 0, 0, 0)
        self.vehicle.send_mavlink(msg)

    @abstractmethod
    def run(self):
        """
        Executes the mission logic.
        """
        pass
