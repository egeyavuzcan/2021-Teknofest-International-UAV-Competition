"""Detection package providing base Detector and implementations."""
from .base import Detector
from .red_color_detector import RedColorDetector

__all__ = ['Detector', 'RedColorDetector']
