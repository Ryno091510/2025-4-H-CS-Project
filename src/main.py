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

class ClawStates():
    OPEN = 'open'
    CLOSED = 'closed'

class ArmHeights(int):
    FLOOR = 0
    LEVEL_1 = 300
    LEVEL_2 = 490
    LEVEL_3 = 700

class DriveSubsystem():
    def __init__(self, brain: Brain):

        self.brain = brain
        self.gyro = Inertial(Ports.PORT18)

        self.leftMotor = Motor(Ports.PORT11, False)
        self.backMotor = Motor(Ports.PORT19, True)
        self.rightMotor = Motor(Ports.PORT20, True)
        self.frontMotor = Motor(Ports.PORT1, False)

        self.leftMotor.set_stopping(HOLD)
        self.backMotor.set_stopping(HOLD)
        self.rightMotor.set_stopping(HOLD)
        self.frontMotor.set_stopping(HOLD)

        self.turningToTarget = False
        self.targetHeading = 0
        self.heading = 0

        self.turnSpeed = 0
        self.yMoveSpeed = 0
        self.xMoveSpeed = 0

    def periodic(self):
        self.heading = self.gyro.heading()
        motorSpeeds = [0.0, 0.0, 0.0, 0.0]

        if abs(self.heading-self.targetHeading) < 10: activeTurnSpeed = 5
        else: activeTurnSpeed = 30
        if self.turningToTarget:
            if abs(self.heading - self.targetHeading) <= 0.5: 
                self.turningToTarget = False
                self.stopTurn()
            elif abs(self.heading-self.targetHeading) < 180:
                if self.heading > self.targetHeading: self.turn(activeTurnSpeed)
                elif self.heading < self.targetHeading: self.turn(-activeTurnSpeed)
            else:
                if self.heading > self.targetHeading: self.turn(-activeTurnSpeed)
                elif self.heading < self.targetHeading: self.turn(activeTurnSpeed)

        
        motorSpeeds[0] = (-self.turnSpeed + self.yMoveSpeed) / 2
        motorSpeeds[1] = (self.turnSpeed + self.xMoveSpeed) / 2
        motorSpeeds[2] = (self.turnSpeed + self.yMoveSpeed) / 2
        motorSpeeds[3] = (-self.turnSpeed + self.xMoveSpeed) / 2
        
        self.leftMotor.spin(FORWARD, motorSpeeds[0], VelocityUnits.PERCENT)
        self.backMotor.spin(FORWARD, motorSpeeds[1], VelocityUnits.PERCENT)
        self.rightMotor.spin(FORWARD, motorSpeeds[2], VelocityUnits.PERCENT)
        self.frontMotor.spin(FORWARD, motorSpeeds[3], VelocityUnits.PERCENT)

    def turn(self, speed):
        self.turnSpeed = speed

    def stopTurn(self): self.turnSpeed = 0

    def driveFieldOriented(self, vertSpeed, horSpeed):
        headingRadians = math.radians(self.heading)
        sinH = math.sin(headingRadians)
        cosH = math.cos(headingRadians)
        self.yMoveSpeed = ((cosH * vertSpeed) + (sinH * horSpeed)) 
        self.xMoveSpeed = (-(sinH * vertSpeed) + (cosH * horSpeed))
    
    def driveRobotOriented(self, vertSpeed, horSpeed):
        self.yMoveSpeed = vertSpeed
        self.xMoveSpeed = horSpeed
    
    def stopDrive(self):
        self.yMoveSpeed = 0
        self.xMoveSpeed = 0

    def turnToHeading(self, target):
        self.turningToTarget = True
        self.targetHeading = target
    
    def driveInDirection(self, direction, magnitude):
        dRadians = math.radians(direction)
        cosD = math.cos(dRadians)
        sinD = math.sin(dRadians)
        self.driveFieldOriented(sinD*magnitude, cosD*magnitude)

class VisionSubsystem():
    def __init__(self, dSubsystem: DriveSubsystem, brain: Brain):
        self.lightBlueColor = Colordesc(1, 45, 109, 108, 1, 0.1)

        self.visionSensor = AiVision(Ports.PORT10, self.lightBlueColor)

        self.visionSensor.start_awb()
        self.visionSensor.tag_detection(True)
        self.visionSensor.model_detection(False)
        self.visionSensor.color_detection(True, True)
        
        self.tags = [Tagdesc(0), Tagdesc(1), Tagdesc(2), Tagdesc(3), Tagdesc(4), Tagdesc(5),]

        self.tagSnapshots = [
            self.visionSensor.take_snapshot(self.tags[0]),
            self.visionSensor.take_snapshot(self.tags[1]),
            self.visionSensor.take_snapshot(self.tags[2]),
            self.visionSensor.take_snapshot(self.tags[3]),
            self.visionSensor.take_snapshot(self.tags[4]),
            self.visionSensor.take_snapshot(self.tags[5])
        ]

        self.driveSubsystem = dSubsystem
        self.brain = brain

        self.scanning = False
        self.tagHeadings = [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        ]
        self.tagHeadingLists = [
            [],
            [],
            [],
            [],
            [],
            []
        ]

    def periodic(self):
        if self.visionSensor.installed():
            for i in range(6):
                self.tagSnapshots[i] = self.visionSensor.take_snapshot(self.tags[i])
                
                if self.scanning:
                    for object in self.tagSnapshots[i]:
                        if object.id is not None and 150 < object.centerX < 170:
                            self.tagHeadingLists[object.id].append(self.driveSubsystem.heading-10)
    
    def averageHeadings(self):
        for i in range(6):
            for data in self.tagHeadingLists[i]:
                self.tagHeadings[i] += data
            if len(self.tagHeadingLists[i]) > 0:
                self.tagHeadings[i] /= len(self.tagHeadingLists[i])

        self.tagHeadingLists = [
            [],
            [],
            [],
            [],
            [],
            []
        ]

class ClawSubsystem():
    def __init__(self, brain: Brain):
        self.brain = brain
        self.clawMotor = Motor(Ports.PORT3)
        self.armMotor = Motor(Ports.PORT8, True)

        self.clawMotor.set_stopping(HOLD)
        self.armMotor.set_stopping(HOLD)

        self.armDownLimit = Bumper(self.brain.three_wire_port.b)

        self.targetClawState = ClawStates.OPEN
        self.currentClawState = ClawStates.OPEN

        self.targetArmHeight = ArmHeights.FLOOR

    def periodic(self):
        clawMotorSpeed = 0.0
        armMotorSpeed = 0.0

        if self.targetClawState is ClawStates.OPEN and self.currentClawState is ClawStates.CLOSED:
            clawMotorSpeed = -50
        if self.targetClawState is ClawStates.CLOSED and self.currentClawState is ClawStates.OPEN:
            clawMotorSpeed = 50
        if self.clawMotor.position() > 190 and clawMotorSpeed > 0:
            self.currentClawState = self.targetClawState
        if self.clawMotor.position() < 10 and clawMotorSpeed < 0:
            self.currentClawState = self.targetClawState
        
        slowArmSpeed = abs(self.armMotor.position() - self.targetArmHeight)

        if self.armMotor.position() > self.targetArmHeight+10: 
            armMotorSpeed = -50
        if self.armMotor.position() < self.targetArmHeight-10:
            armMotorSpeed = 50

        if (self.armDownLimit.pressing() or self.armMotor.position() < -5) and armMotorSpeed > 0:
            armMotorSpeed = 0.0
        if self.armMotor.position() < -700 and armMotorSpeed < 0:
            armMotorSpeed = 0.0
        
        self.clawMotor.spin(FORWARD, clawMotorSpeed, VelocityUnits.PERCENT)
        self.armMotor.spin(FORWARD, armMotorSpeed, VelocityUnits.PERCENT)


    def openClaw(self):
        self.targetClawState = ClawStates.OPEN

    def closeClaw(self):
        self.targetClawState = ClawStates.CLOSED
    
    def toggleClaw(self):
        if self.targetClawState == ClawStates.OPEN:
            self.targetClawState = ClawStates.CLOSED
        elif self.targetClawState == ClawStates.CLOSED:
            self.targetClawState = ClawStates.OPEN

    def raiseArm(self):
        if self.targetArmHeight == ArmHeights.FLOOR:
            self.targetArmHeight = ArmHeights.LEVEL_1

        elif self.targetArmHeight == ArmHeights.LEVEL_1:
            self.targetArmHeight = ArmHeights.LEVEL_2

        elif self.targetArmHeight == ArmHeights.LEVEL_2:
            self.targetArmHeight = ArmHeights.LEVEL_3
    
    def lowerArm(self):
        if self.targetArmHeight == ArmHeights.LEVEL_3:
            self.targetArmHeight = ArmHeights.LEVEL_2

        elif self.targetArmHeight == ArmHeights.LEVEL_2:
            self.targetArmHeight = ArmHeights.LEVEL_1

        elif self.targetArmHeight == ArmHeights.LEVEL_1:
            self.targetArmHeight = ArmHeights.FLOOR
    
    def setArmHeight(self, height: ArmHeights):
        self.targetArmHeight = height


def DriveToTag(id, driveSubsystem: DriveSubsystem, visionSubsystem: VisionSubsystem):
    driveSubsystem.turnToHeading(visionSubsystem.tagHeadings[id])
    # driveSubsystem.driveInDirection(visionSubsystem.tagHeadings[id], 40)
    while driveSubsystem.turningToTarget:
        controller.screen.clear_screen()
        controller.screen.set_cursor(1, 1)
        controller.screen.print(driveSubsystem.heading)
        controller.screen.next_row()
        controller.screen.print("")
        controller.screen.next_row()
        controller.screen.print(visionSubsystem.tagHeadings[1])
    driveSubsystem.driveRobotOriented(80, 0)
    driveSubsystem.stopDrive()

def grabFromFloor(clawSubsystem: ClawSubsystem):
    pass


brain = Brain()
controller = Controller()

driveSubsystem = DriveSubsystem(brain)
visionSubsystem = VisionSubsystem(driveSubsystem, brain)
clawSubsystem = ClawSubsystem(brain)

controllerBEvent = controller.buttonB.pressed(clawSubsystem.toggleClaw)
controllerR1Event = controller.buttonR1.pressed(clawSubsystem.raiseArm)
controllerR2Event = controller.buttonR2.pressed(clawSubsystem.lowerArm)

visionSubsystem.scanning = False

iterations = 0

def periodicThreadRun():
    while True:
        driveSubsystem.periodic()
        visionSubsystem.periodic()
        clawSubsystem.periodic()
        wait(20, MSEC)

def controlThreadRun():

    while True:
        if visionSubsystem.scanning:
            driveSubsystem.turn(-10)

        if driveSubsystem.heading > 355 and visionSubsystem.scanning:
            visionSubsystem.scanning = False
            driveSubsystem.stopTurn()
            visionSubsystem.averageHeadings()
        if controller.buttonA.pressing():
            DriveToTag(1, driveSubsystem, visionSubsystem)
        if controller.buttonB.pressing():
            driveSubsystem.stopDrive()

        controller.screen.clear_screen()
        controller.screen.set_cursor(1, 1)
        controller.screen.print(driveSubsystem.heading)
        controller.screen.next_row()
        controller.screen.print(clawSubsystem.targetArmHeight)
        controller.screen.next_row()
        controller.screen.print(clawSubsystem.armMotor.position()) # visionSubsystem.tagHeadings[1])
        wait(50, MSEC)

wait(3, SECONDS)

mainThread = Thread(controlThreadRun)
periodicThread = Thread(periodicThreadRun)
