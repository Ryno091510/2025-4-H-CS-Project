# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Ryan Mejeur                                                  #
# 	Created:      6/16/2025, 12:21:48 PM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *
import math


class DriveSubsystem():
    def __init__(self, brain):

        self.brain = brain
        self.gyro = Inertial(Ports.PORT16)
        self.leftMotor = Motor(Ports.PORT11, False)
        self.backMotor = Motor(Ports.PORT19, True)
        self.rightMotor = Motor(Ports.PORT20, True)

    def run(self):
        motorSpeeds = [0, 0, 0]
        
        self.leftMotor.spin(FORWARD, motorSpeeds[0])
        self.backMotor.spin(FORWARD, motorSpeeds[1])
        self.rightMotor.spin(FORWARD, motorSpeeds[2])

    def turn(self, speed):
        pass

    def drive(self, vertSpeed, horSpeed):
        pass


brain = Brain()

driveSubsystem = DriveSubsystem(brain)


while True:
    driveSubsystem.run()