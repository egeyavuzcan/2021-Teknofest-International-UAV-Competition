"""
Base class for command-line applications.
"""
from abc import ABC, abstractmethod


class BaseApp(ABC):
    """
    Defines the interface for CLI applications.
    """

    @abstractmethod
    def run(self):
        """
        Execute the application logic.
        """
        pass
