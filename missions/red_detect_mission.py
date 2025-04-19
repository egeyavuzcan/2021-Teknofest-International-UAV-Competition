"""
Mission module for red detection with autonomous flight (Mission 2).
"""
import cv2
from dronekit import VehicleMode
import time
from .flight_mission import FlightMission
from detection.red_color_detector import RedColorDetector


class RedDetectMission(FlightMission):
    """
    Detect red objects and control the drone based on detection.

    Usage:
        mission = RedDetectMission(conn_str)
        mission.arm_and_takeoff(altitude)
        mission.run()
    """

    def __init__(self, connection_str: str, baud: int = 115200,
                 gnd_speed: float = 0.1, field_threshold: float = 100):
        super().__init__(connection_str, baud, gnd_speed)
        self.field_threshold = field_threshold
        self.detector = RedColorDetector()
        self.degree = 0

    def run(self, camera_index: int = 0):
        """
        Capture frames, detect red, and perform yaw control.
        Press ESC to exit.
        """
        cap = cv2.VideoCapture(camera_index)
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                # apply red-color detection
                output = self.detector.detect(frame)
                # TODO: extract contours, compute center, area, and control vehicle
                cv2.imshow("RedDetectMission", output)
                if cv2.waitKey(1) & 0xFF == 27:
                    break
        finally:
            cap.release()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
        description="Mission 2: Red detection with autonomous flight"
    )
    parser.add_argument("--conn", type=str, default="127.0.0.1:14551",
                        help="MAVLink connection string")
    parser.add_argument("--field", type=float, default=100,
                        help="Contour area threshold")
    parser.add_argument("--alt", type=float, default=10,
                        help="Takeoff altitude")
    args = parser.parse_args()
    mission = RedDetectMission(args.conn, field_threshold=args.field)
    mission.arm_and_takeoff(args.alt)
    mission.run()
