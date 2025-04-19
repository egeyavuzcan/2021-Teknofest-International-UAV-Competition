"""
Module for simple red detection mission (Mission1).
"""
import cv2
import time
from detection.red_color_detector import RedColorDetector


class SimpleDetectionMission:
    """
    Real-time red detection with FPS overlay and optional recording.

    Usage:
        mission = SimpleDetectionMission(camera_index=0, record=True, output_file='output.avi')
        mission.run()
    """

    def __init__(self, camera_index: int = 0, record: bool = False, output_file: str = 'output.avi'):
        self.camera_index = camera_index
        self.record = record
        self.output_file = output_file
        self.detector = RedColorDetector()

    def run(self):
        """
        Runs the detection loop until ESC is pressed.
        """
        cap = cv2.VideoCapture(self.camera_index)
        prev_time = time.time()

        if self.record:
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            writer = cv2.VideoWriter(
                self.output_file,
                fourcc,
                20.0,
                (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            )

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break

                # Rotate frame and overlay FPS
                frame = cv2.rotate(frame, cv2.ROTATE_180)
                curr_time = time.time()
                fps = int(1.0 / (curr_time - prev_time)) if curr_time != prev_time else 0
                prev_time = curr_time
                cv2.putText(frame, f"FPS: {fps}", (20, 70), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)

                # Detect red and show output
                output = self.detector.detect(frame)
                cv2.imshow("SimpleDetectionMission", output)

                if self.record:
                    writer.write(output)

                if cv2.waitKey(1) & 0xFF == 27:
                    break
        finally:
            cap.release()
            if self.record:
                writer.release()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Mission1: Simple red detection with FPS and optional recording")
    parser.add_argument("--camera", type=int, default=0, help="Camera index")
    parser.add_argument("--record", action="store_true", help="Enable recording")
    parser.add_argument("--output", type=str, default="output.avi", help="Output AVI filename")
    args = parser.parse_args()

    mission = SimpleDetectionMission(camera_index=args.camera, record=args.record, output_file=args.output)
    mission.run()
