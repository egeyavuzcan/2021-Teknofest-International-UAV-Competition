"""
Concrete detector for red-color objects based on HSV thresholding.
"""
import numpy as np
import cv2
from .base import Detector


class RedColorDetector(Detector):
    """
    Detects red-colored regions in a video frame.

    Uses Gaussian blur, HSV thresholding, and contour drawing.
    """

    def __init__(self, low_hsv=(155, 105, 80), high_hsv=(179, 255, 255), blur_ksize=(11, 11), morph_iters=2):
        self.low_hsv = np.array(low_hsv)
        self.high_hsv = np.array(high_hsv)
        self.blur_ksize = blur_ksize
        self.morph_iters = morph_iters

    def detect(self, frame: np.ndarray) -> np.ndarray:
        # Rotate and blur
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        blurred = cv2.GaussianBlur(frame, self.blur_ksize, 3)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Threshold red and clean up mask
        mask = cv2.inRange(hsv, self.low_hsv, self.high_hsv)
        mask = cv2.erode(mask, None, iterations=self.morph_iters)
        mask = cv2.dilate(mask, None, iterations=self.morph_iters)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        # Find contours and draw
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        output = frame.copy()
        if contours:
            c = max(contours, key=cv2.contourArea)
            box = cv2.boxPoints(cv2.minAreaRect(c))
            box = box.astype(int)
            cv2.drawContours(output, [box], 0, (0, 255, 255), 2)

        return output


if __name__ == "__main__":
    # Example usage
    detector = RedColorDetector()
    detector.run()  # Press ESC to exit
