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
import math, random, time


class DriveSubsystem():
    def __init__(self, brain: Brain):

        self.brain = brain
        self.gyro = Inertial(Ports.PORT18)

        self.leftMotor = Motor(Ports.PORT11, False)
        self.backMotor = Motor(Ports.PORT19, True)
        self.rightMotor = Motor(Ports.PORT20, True)
        self.frontMotor = Motor(Ports.PORT4, False)

        self.leftMotor.set_stopping(HOLD)
        self.backMotor.set_stopping(HOLD)
        self.rightMotor.set_stopping(HOLD)
        self.frontMotor.set_stopping(HOLD)

        self.turningToTarget = False
        self.targetHeading = 0
        self.heading = 0

        self.turnSpeed = 0
        self.sideDriveSpeed = 0
        self.horDriveSpeed = 0

    def run(self):
        self.heading = self.gyro.heading()
        motorSpeeds = [0, 0, 0, 0]

        if self.turningToTarget:
            if self.heading > self.targetHeading: self.turn(2.5)
            elif self.heading < self.targetHeading: self.turn(-2.5)
            if abs(self.heading - self.targetHeading) <= 1: 
                self.turningToTarget = False
                self.stopTurn()
        
        motorSpeeds[0] = motorSpeeds[3] = -self.turnSpeed
        motorSpeeds[1] = motorSpeeds[2] = self.turnSpeed
        motorSpeeds[0] = motorSpeeds[2] = self.sideDriveSpeed
        motorSpeeds[1] = motorSpeeds[3] = self.backDriveSpeed
        
        self.leftMotor.spin(FORWARD, motorSpeeds[0], VelocityUnits.PERCENT)
        self.backMotor.spin(FORWARD, motorSpeeds[1], VelocityUnits.PERCENT)
        self.rightMotor.spin(FORWARD, motorSpeeds[2], VelocityUnits.PERCENT)
        self.frontMotor.spin(FORWARD, motorSpeeds[3], VelocityUnits.PERCENT)

    def turn(self, speed):
        self.turnSpeed = speed

    def stopTurn(self): self.turnSpeed = 0

    def drive(self, vertSpeed, horSpeed):
        headingRadians = math.radians(self.heading)
        sinH = math.sin(headingRadians)
        cosH = math.cos(headingRadians)
        self.sideDriveSpeed = ((cosH * vertSpeed) + (sinH * horSpeed)) 
        self.backDriveSpeed = (-(sinH * vertSpeed) + (cosH * horSpeed))

    def turnToHeading(self, target):
        self.turningToTarget = True
        self.targetHeading = target


brain = Brain()
controller = Controller()

driveSubsystem = DriveSubsystem(brain)

time.sleep(3)

driveSubsystem.turnToHeading(120)

while True:
    driveSubsystem.drive(controller.axis3.position(), controller.axis4.position())
    driveSubsystem.run()