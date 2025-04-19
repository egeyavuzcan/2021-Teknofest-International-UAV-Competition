"""
Abstract base class for all detectors.
"""
from abc import ABC, abstractmethod
import numpy as np


class Detector(ABC):
    """
    Defines the interface for detector implementations.

    Subclasses must override the detect method.
    """

    @abstractmethod
    def detect(self, frame: np.ndarray) -> np.ndarray:
        """
        Process a single video frame and return an annotated image or mask.

        Args:
            frame (np.ndarray): Input image in BGR color space.

        Returns:
            np.ndarray: Annotated image or mask highlighting detections.
        """
        pass

    def run(self, camera_index: int = 0, window_name: str = "Detection"):
        """
        Capture video frames from a camera and apply detection in real time.

        Args:
            camera_index (int): OpenCV camera index (default=0).
            window_name (str): Title of the display window.
        """
        import cv2

        cap = cv2.VideoCapture(camera_index)
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                output = self.detect(frame)
                cv2.imshow(window_name, output)
                if cv2.waitKey(1) & 0xFF == 27:
                    break
        finally:
            cap.release()
            cv2.destroyAllWindows()
