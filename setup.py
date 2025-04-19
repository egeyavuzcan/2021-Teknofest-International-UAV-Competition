from setuptools import setup, find_packages

setup(
    name='tekno_uav_competition',
    version='0.1.0',
    description='Red color detection and autonomous UAV missions for Teknofest 2021',
    author='Your Name',
    author_email='you@example.com',
    packages=find_packages(where='.'),
    install_requires=[
        'numpy>=1.19.0',
        'opencv-python>=4.5.0',
        'dronekit>=2.0.0',
        'pymavlink>=2.4.0',
        'RPi.GPIO>=0.7.0; platform_system=="Linux"',
    ],
    python_requires='>=3.6',
    classifiers=[
        'Programming Language :: Python :: 3',
        'Operating System :: OS Independent',
    ],
)
