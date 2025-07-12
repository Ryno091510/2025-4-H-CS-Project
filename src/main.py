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


class ClawStates():
    OPEN = 'open'
    CLOSED = 'closed'

class ArmHeights(int):
    FLOOR = 10
    LEVEL_1 = 300
    LEVEL_2 = 475
    LEVEL_3 = 695

#region Subsystems

class DriveSubsystem():                                                         # Defines the DriveSubsytem class
    '''
    A subsystem to control the robot's drivetrain.
    '''
    def __init__(self, brain: Brain):
        '''
        Initializes the subsystem as an object and defines object-wide variables.

        :param Brain brain: The Vex V5 Brain object
        '''

        self.brain = brain                                                      # takes the 'brain' argument of this function and makes it available to the rest of the class

        self.gyro = Inertial(Ports.PORT18)                                      # Initializes the gyro sensor to detect the robot's heading

        self.leftMotor = Motor(Ports.PORT11, False)                             # |
        self.backMotor = Motor(Ports.PORT19, True)                              #  \ Initializes all of the drive motors, with the first argument being the port,
        self.rightMotor = Motor(Ports.PORT20, True)                             #  / and the second determining whether or not the motor is reversed
        self.frontMotor = Motor(Ports.PORT1, False)                             # |

        self.leftMotor.set_stopping(HOLD)                                       # |
        self.backMotor.set_stopping(HOLD)                                       #  > Tells all of the drive motors to hold their position when stopped
        self.rightMotor.set_stopping(HOLD)                                      # |
        self.frontMotor.set_stopping(HOLD)                                      # |

        self.turningToTarget = False                                            # Defines the turningToTarget variable
        self.targetHeading = 0                                                  # Defines the targetHeading varaible
        self.heading = 0.0                                                      # Defines the heading variable

        self.turnSpeed = int(0)                                                 # 
        self.yMoveSpeed = 0                                                     # 
        self.xMoveSpeed = 0                                                     # 

    def periodic(self):
        '''
        Runs every twenty milliseconds.

        Used for updating the robot's heading in the code and for controlling the motors.
        '''
        self.heading = self.gyro.heading()                                      # Updates the heading to the reading from the gyro
        motorSpeeds = [0.0, 0.0, 0.0, 0.0]                                      # Creates the function-only variable to place the motor speeds into

        if abs(self.heading-self.targetHeading) < 10: activeTurnSpeed = 5       # Slows turning when closer to the target heading
        else: activeTurnSpeed = 30                                              # Otherwise, turn faster

        if self.turningToTarget:                                                # If attempting to turn to a specific heading, run the following

            if abs(self.heading - self.targetHeading) <= 0.75:                  # If within 3/4 a degree of the target, run below
                self.turningToTarget = False                                    # Set turningToTarget to false
                self.stopTurn()                                                 # Stop turning
            
            elif abs(self.heading-self.targetHeading) < 180:                            # If within 180 degrees of target angle
                if self.heading > self.targetHeading: self.turn(activeTurnSpeed)        # If clockwise of target angle, turn normally
                elif self.heading < self.targetHeading: self.turn(-activeTurnSpeed)     # If counterclockwise of target angle, turn reversed

            else:                                                                       # Otherwise
                if self.heading > self.targetHeading: self.turn(-activeTurnSpeed)       # If clockwise of target angle, turn reversed
                elif self.heading < self.targetHeading: self.turn(activeTurnSpeed)      # If counterclocwise of target angle, turn normally

        
        motorSpeeds[0] = (-self.turnSpeed + self.yMoveSpeed) / 2                # Setting motor speeds to averages of turn speed and drive speeds
        motorSpeeds[1] = (self.turnSpeed + self.xMoveSpeed) / 2                 #   |
        motorSpeeds[2] = (self.turnSpeed + self.yMoveSpeed) / 2                 #   |
        motorSpeeds[3] = (-self.turnSpeed + self.xMoveSpeed) / 2                #   |
        
        self.leftMotor.spin(FORWARD, motorSpeeds[0], VelocityUnits.PERCENT)     # Sending the target motors speeds to the motors
        self.backMotor.spin(FORWARD, motorSpeeds[1], VelocityUnits.PERCENT)     #   |
        self.rightMotor.spin(FORWARD, motorSpeeds[2], VelocityUnits.PERCENT)    #   |
        self.frontMotor.spin(FORWARD, motorSpeeds[3], VelocityUnits.PERCENT)    #   |

    def turn(self, speed: int):
        '''
        Starts the robot turning with the desired speed and direction.
        
        :param int speed: The speed to turn the robot at
        '''
        self.turnSpeed = speed                                                  # Changes the turnSpeed variable to match the input speed

    def stopTurn(self):
        '''
        Stops the robot's turn
        ''' 
        self.turnSpeed = 0                                                      # Sets the turnSpeed variable to 0

    def driveFieldOriented(self, vertSpeed, horSpeed):
        '''
        Commands the robot to drive field-oriented, which means that one direction is always forward,
        no matter which way the robot is facing.

        :param int vertSpeed: The speed the robot should move forward and backward relative the original direction
        :param int horSpeed: The speed the robot should move left and right relative the original direction
        '''
        headingRadians = math.radians(self.heading)                             # Gets the robot's current heading in radians
        sinH = math.sin(headingRadians)                                         # Gets the sine of the robot's heading
        cosH = math.cos(headingRadians)                                         # Gets the cosine of the robot's heading
        self.yMoveSpeed = ((cosH * vertSpeed) + (sinH * horSpeed))              # Get the y movement speed from multiplication of the cosine, sine, and input variables
        self.xMoveSpeed = (-(sinH * vertSpeed) + (cosH * horSpeed))             # Get the x movement speed from multiplication of the sine, cosine, and input variables
    
    def driveRobotOriented(self, vertSpeed, horSpeed):
        '''
        Commands the robot to drive oriented to the direction it is facing
        '''
        self.yMoveSpeed = vertSpeed                                             # 
        self.xMoveSpeed = horSpeed                                              # 
    
    def stopDrive(self):
        self.yMoveSpeed = 0                                                     # 
        self.xMoveSpeed = 0                                                     # 

    def turnToHeading(self, target):
        self.turningToTarget = True                                             # 
        self.targetHeading = target                                             # 

    def resetHeading(self):
        self.gyro.reset_heading()                                               # 

class VisionSubsystem():
    def __init__(self, dSubsystem: DriveSubsystem, brain: Brain):
        self.tags = [
            Tagdesc(0),
            Tagdesc(1),
            Tagdesc(2),
            Tagdesc(3),
            Tagdesc(4),
            Tagdesc(5)]
        self.colors = [
            Colordesc(1, 71, 200, 200, 20, 0.4)
        ]

        self.visionSensor = AiVision(Ports.PORT10, self.colors[0])

        self.visionSensor.start_awb()
        self.visionSensor.tag_detection(True)
        self.visionSensor.model_detection(False)
        self.visionSensor.color_detection(True, True)

        self.tagSnapshots = [
            self.visionSensor.take_snapshot(self.tags[0]),
            self.visionSensor.take_snapshot(self.tags[1]),
            self.visionSensor.take_snapshot(self.tags[2]),
            self.visionSensor.take_snapshot(self.tags[3]),
            self.visionSensor.take_snapshot(self.tags[4]),
            self.visionSensor.take_snapshot(self.tags[5])
        ]

        self.colorSnapshots = [
            self.visionSensor.take_snapshot(self.colors[0])
        ]

        self.driveSubsystem = dSubsystem
        self.brain = brain

        self.scanning = True
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
            for i in range(len(self.tagSnapshots)):
                self.tagSnapshots[i] = self.visionSensor.take_snapshot(self.tags[i])
                
                if self.scanning:
                    for object in self.tagSnapshots[i]:
                        if object.id is not None and 150 < object.centerX < 170:
                            self.tagHeadingLists[i].append(self.driveSubsystem.heading-10)
            
            for i in range(len(self.colorSnapshots)):
                self.colorSnapshots[i] = self.visionSensor.take_snapshot(self.colors[i])

            self.towerHeight = len(self.colorSnapshots[0])
    
    def averageHeadings(self):
        for i in range(6):
            self.tagHeadings[i] = 0
            for data in self.tagHeadingLists[i]:
                self.tagHeadings[i] += data
            if len(self.tagHeadingLists[i]) > 0:
                self.tagHeadings[i] /= len(self.tagHeadingLists[i])

        self.tagHeadingLists = [
            [],                                                                 # 
            [],                                                                 # 
            [],                                                                 # 
            [],                                                                 # 
            [],                                                                 # 
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
        self.armAtTarget = True

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

        if self.targetArmHeight-7.5 < self.armMotor.position() < self.targetArmHeight+7.5:
            self.armAtTarget = True
        else:
            self.armAtTarget = False
        
        if not self.armAtTarget:
            armMotorSpeed = math.copysign(40, self.targetArmHeight - self.armMotor.position())

        if abs(self.armMotor.position()-self.targetArmHeight) <= 75:
            armMotorSpeed = armMotorSpeed/6
        elif abs(self.armMotor.position()-self.targetArmHeight) <= 40:
            armMotorSpeed = armMotorSpeed/8

        if self.armMotor.position() < 0 and armMotorSpeed < 0:
            armMotorSpeed = 0.0
        if self.armMotor.position() > 700 and armMotorSpeed > 0:
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
    
    def setArmHeight(self, height: int):
        self.targetArmHeight = height

#endregion Subsystems

#region Commands

def DriveToTag(id, speed, proximity, dSubsystem: DriveSubsystem, vSubsystem: VisionSubsystem):
    dSubsystem.turnToHeading(vSubsystem.tagHeadings[id])
    # driveSubsystem.driveInDirection(visionSubsystem.tagHeadings[id], 80)
    while dSubsystem.turningToTarget: pass
    dSubsystem.driveRobotOriented(speed, 0)
    while vSubsystem.tagSnapshots[1][0].width < proximity: controller.screen.set_cursor(2,1); controller.screen.print(vSubsystem.towerHeight)
    dSubsystem.stopDrive()

def CollectFromStack(id, dSubsystem: DriveSubsystem, vSubsystem: VisionSubsystem, cSubsystem: ClawSubsystem):
    dSubsystem.turnToHeading(vSubsystem.tagHeadings[id])
    while dSubsystem.turningToTarget: wait(20, MSEC)
    wait(250, MSEC)
    proximity = 100
    if vSubsystem.towerHeight == 0: cSubsystem.setArmHeight(ArmHeights.FLOOR); proximity = 30
    elif vSubsystem.towerHeight == 1: cSubsystem.setArmHeight(ArmHeights.LEVEL_1); proximity = 25
    elif vSubsystem.towerHeight == 2: cSubsystem.setArmHeight(ArmHeights.LEVEL_2); proximity = 23
    elif vSubsystem.towerHeight == 3: cSubsystem.setArmHeight(ArmHeights.LEVEL_3); proximity = 30
    wait(50, MSEC)
    while not cSubsystem.armAtTarget: pass
    dSubsystem.driveRobotOriented(50, 0)
    while vSubsystem.tagSnapshots[id][0].width < proximity: pass
    dSubsystem.stopDrive()
    cSubsystem.closeClaw()
    while cSubsystem.currentClawState == ClawStates.OPEN: pass
    dSubsystem.driveRobotOriented(-50, 0)
    wait(1, SECONDS)
    cSubsystem.setArmHeight(ArmHeights.FLOOR)
    dSubsystem.stopDrive()

def DropInStack(id, dSubsystem: DriveSubsystem, vSubsystem: VisionSubsystem, cSubsystem: ClawSubsystem):
    dSubsystem.turnToHeading(vSubsystem.tagHeadings[id]); print('start turn')
    step = 0
    while dSubsystem.turningToTarget: wait(20, MSEC); step += 1; print('turning', step)
    wait(250, MSEC); print('waiting 250')
    proximity = 100
    if vSubsystem.towerHeight == 0: cSubsystem.setArmHeight(ArmHeights.FLOOR); proximity = 30; print('height 0')
    elif vSubsystem.towerHeight == 1: cSubsystem.setArmHeight(ArmHeights.LEVEL_1); proximity = 25; print('height 1')
    elif vSubsystem.towerHeight == 2: cSubsystem.setArmHeight(ArmHeights.LEVEL_2); proximity = 25; print('height 2')
    elif vSubsystem.towerHeight == 3: cSubsystem.setArmHeight(ArmHeights.LEVEL_3); proximity = 30; print('height 3')
    wait(50, MSEC); print('waiting 50')
    step = 0
    while not cSubsystem.armAtTarget: wait(20, MSEC); step += 1; print('lifting', step)
    dSubsystem.driveRobotOriented(50, 0); print('drive front')
    step = 0
    while vSubsystem.tagSnapshots[id][0].width < proximity: wait(20, MSEC); step += 1; print('driving', step)
    dSubsystem.stopDrive(); print('stop')
    cSubsystem.openClaw(); print('open')
    step = 0
    while cSubsystem.currentClawState == ClawStates.CLOSED: wait(20, MSEC); step += 1; print('opening', step)
    dSubsystem.driveRobotOriented(-50, 0); print('drive back')
    wait(1, SECONDS); print('wait 1 sec')
    cSubsystem.setArmHeight(ArmHeights.FLOOR); print('arm down')
    dSubsystem.stopDrive(); print('stop')

def ScanForTag(toZero: bool, dSubsystem: DriveSubsystem, vSubsystem: VisionSubsystem):
    if toZero:
        dSubsystem.turnToHeading(5)
        while dSubsystem.turningToTarget: pass
        wait(100, MSEC)
    dSubsystem.turn(-10)
    vSubsystem.scanning = True
    steps = 0
    while True:
        steps += 1
        if dSubsystem.heading > 355 and steps > 50:
            vSubsystem.scanning = False
            dSubsystem.stopTurn()
            vSubsystem.averageHeadings()
            break
        wait(20, MSEC)

#endregion Commands


brain = Brain()
controller = Controller()

driveSubsystem = DriveSubsystem(brain)
visionSubsystem = VisionSubsystem(driveSubsystem, brain)
clawSubsystem = ClawSubsystem(brain)



controllerL1Event = controller.buttonL1.pressed(clawSubsystem.toggleClaw)
controllerR1Event = controller.buttonR1.pressed(clawSubsystem.raiseArm)
controllerR2Event = controller.buttonR2.pressed(clawSubsystem.lowerArm)

emergencyStopEvent = controller.buttonLeft.pressed(brain.program_stop)


def periodicThread():
    while True:
        driveSubsystem.periodic()
        visionSubsystem.periodic()
        clawSubsystem.periodic()
        displayDebugData()
        wait(20, MSEC)

def displayDebugData():
    controller.screen.clear_screen()
    controller.screen.set_cursor(1, 1)
    controller.screen.print(clawSubsystem.targetArmHeight)
    controller.screen.next_row()
    controller.screen.print(clawSubsystem.armMotor.position())
    controller.screen.next_row()
    controller.screen.print(str(clawSubsystem.armAtTarget))


# while True:
    
#     if controller.buttonA.pressing():
#         CollectFromStack(1, driveSubsystem, visionSubsystem, clawSubsystem)
#     if controller.buttonB.pressing():
#         DropInStack(1, driveSubsystem, visionSubsystem, clawSubsystem)

#     controller.screen.clear_screen()
#     controller.screen.set_cursor(1, 1)
#     controller.screen.print(driveSubsystem.heading)
#     controller.screen.next_row()
#     controller.screen.print(visionSubsystem.towerHeight)
#     controller.screen.next_row()
#     controller.screen.print(visionSubsystem.tagHeadings[1])
#     wait(50, MSEC)

wait(2, SECONDS)

periodic = Thread(periodicThread)

ScanForTag(False, driveSubsystem, visionSubsystem)

CollectFromStack(0, driveSubsystem, visionSubsystem, clawSubsystem)

ScanForTag(True, driveSubsystem,visionSubsystem)

DropInStack(1, driveSubsystem, visionSubsystem, clawSubsystem)
