#!/usr/bin/env python3
import numpy as np

class Pose_Gain:
    def __init__(self) -> None:
        self.kp = np.array([
            [1500, 0, 0, 0, 0],
            [0, 1000, 0, 0, 0],
            [0, 0, 5000, 0, 0]
        ])
        self.kd = np.array([
            [3750, 0, 0, 0, 0],
            [0, 3000, 0, 0, 0],
            [0, 0, 18847.65625, 0, 0]
            ])

        self.ki = np.array([
            [0.0, 0, 0, 0, 0],
            [0, 0.0, 0, 0, 0],
            [0, 0, 0.0, 0, 0]
        ])

        self.scale = 1
        

class Force_Gain:
    def __init__(self) -> None:
        self.kp = np.array([
            [1, 0, 0, 0, 0],
            [0, 0.8, 0, 0, 0],
            [0, 0, 5000, 0, 0]
        ])
        self.kd = np.array([
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 18847.65625, 0, 0]
            ])

        self.ki = np.array([
            [0.0, 0, 0, 0, 0],
            [0, 0.0, 0, 0, 0],
            [0, 0, 0.0, 0, 0]
        ])

        self.scale = 1

class Force_Gain_GLF:
    def __init__(self) -> None:
        self.kp = np.array([
            [100, 0, 0, 0, 0],
            [0, 75, 0, 0, 0],
            [0, 0, 150, 0, 0]
        ])
        self.kd = np.array([
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 56.125, 0, 0]
            ])

        self.ki = np.array([
            [0.0, 0, 0, 0, 0],
            [0, 0.0, 0, 0, 0],
            [0, 0, 0.0, 0, 0]
        ])

        self.scale = 2000
        

class Pose_Gain_GLF:
    def __init__(self) -> None:
        self.kp = np.array([
            [50, 0, 0, 0, 0],
            [0, 0 *10, 0, 0, 0],
            [0, 0, 0*75, 0, 0]
        ])
        self.kd = np.array([
            [75, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 20, 0, 0]
            ])

        self.ki = np.array([
            [0.0, 0, 0, 0, 0],
            [0, 0.0, 0, 0, 0],
            [0, 0, 0.0, 0, 0]
        ])

        self.scale = 1000
