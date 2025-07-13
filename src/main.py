# Library imports
from vex import *                                                               # Imports everything from the vex library for the robot to use
import math                                                                     # Imports the math library for various mathmatical features


class ClawStates():
    '''
    Used for simply controlling the claw.
    '''
    OPEN = 'open'                                                               # Defines the open state
    CLOSED = 'closed'                                                           # Defines the closed state

class ArmHeights(int):
    '''
    Used for the arm heights.
    '''
    FLOOR = 10                                                                  # Floor level
    LEVEL_1 = 255                                                               # One box
    LEVEL_2 = 445                                                               # two boxes
    LEVEL_3 = 690                                                               # three boxes

#region Subsystems

class DriveSubsystem():
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

        self.leftMotor = Motor(Ports.PORT12, False)                             # |
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

        self.turnSpeed = 0                                                      # 
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
            
            elif abs(self.heading-self.targetHeading) < 180:                            # If the absolute of difference of the heading and target is less than 180
                if self.heading > self.targetHeading: self.turn(activeTurnSpeed)        # If clockwise of target angle, turn normally
                elif self.heading < self.targetHeading: self.turn(-activeTurnSpeed)     # If counterclockwise of target angle, turn reversed

            else:                                                                       # Otherwise
                if self.heading > self.targetHeading: self.turn(-activeTurnSpeed)       # If clockwise of target angle, turn reversed
                elif self.heading < self.targetHeading: self.turn(activeTurnSpeed)      # If counterclocwise of target angle, turn normally

        
        motorSpeeds[0] = (-self.turnSpeed + self.yMoveSpeed) / 2                # Setting motor speeds to averages of turn speed and drive speeds
        motorSpeeds[1] = (self.turnSpeed + self.xMoveSpeed) / 2                 #   |
        motorSpeeds[2] = (self.turnSpeed + self.yMoveSpeed) / 2                 #   |
        motorSpeeds[3] = (-self.turnSpeed + self.xMoveSpeed) / 2                #   |
        
        self.leftMotor.spin(FORWARD, motorSpeeds[0], VelocityUnits.PERCENT)     # Sending the target motor speeds to the motors
        self.rightMotor.spin(FORWARD, motorSpeeds[2], VelocityUnits.PERCENT)    #   |
        self.backMotor.spin(FORWARD, motorSpeeds[1], VelocityUnits.PERCENT)     #   |
        self.frontMotor.spin(FORWARD, motorSpeeds[3], VelocityUnits.PERCENT)    #   |

    def turn(self, speed: int):
        '''
        Starts the robot turning with the desired speed and direction.
        
        :param int speed: The speed to turn the robot at
        '''
        self.turnSpeed = speed                                                  # Changes the turnSpeed variable to match the input speed

    def stopTurn(self):
        '''
        Stops the robot's turn.
        ''' 
        self.turnSpeed = 0                                                      # Sets the turnSpeed variable to 0

    def driveFieldOriented(self, vertSpeed: int, horSpeed: int):
        '''
        Commands the robot to drive field-oriented, which means that one direction is always forward,
        no matter which way the robot is facing.

        :param int vertSpeed: The speed the robot should move forward and backward relative the original direction
        :param int horSpeed: The speed the robot should move left and right relative the original direction
        '''
        headingRadians = math.radians(self.heading)                             # Gets the robot's current heading in radians
        sinH = math.sin(headingRadians)                                         # Gets the sine of the robot's heading
        cosH = math.cos(headingRadians)                                         # Gets the cosine of the robot's heading
        self.yMoveSpeed = ((cosH * vertSpeed) + (sinH * horSpeed))              # Sets the y movement speed from multiplication of the cosine, sine, and input variables
        self.xMoveSpeed = (-(sinH * vertSpeed) + (cosH * horSpeed))             # Sets the x movement speed from multiplication of the sine, cosine, and input variables
    
    def driveRobotOriented(self, vertSpeed: int, horSpeed: int):
        '''
        Commands the robot to drive oriented to the direction it is facing.

        :param int vertSpeed: The speed the robot should move forward and backward
        :param int horSpeed: The speed the robot should move left and right
        '''
        self.yMoveSpeed = vertSpeed                                             # Sets the y movement speed to the supplied vertical speed
        self.xMoveSpeed = horSpeed                                              # Sets the x movement speed to the supplied horizontal speed
    
    def stopDrive(self):
        '''
        Stops the robot's driving.
        '''
        self.yMoveSpeed = 0                                                     # Sets the y movement speed to 0
        self.xMoveSpeed = 0                                                     # Sets the x movement speed to 0

    def turnToHeading(self, target: float):
        '''
        Commands the robot to face a specified heading.

        :param float target
        '''
        self.turningToTarget = True                                             # Sets the turningToTarget variable to True
        self.targetHeading = target                                             # Sets the target heading to the supplied target

    def resetHeading(self):
        '''
        Resets the gyro on the robot back to zero.
        '''
        self.gyro.reset_heading()                                               # Resets the heading on the gyro

class VisionSubsystem():
    '''
    A subsystem for communicating with the AI Vision Sensor.
    '''
    def __init__(self, dSubsystem: DriveSubsystem, brain: Brain):
        '''
        Initializes the subsystem as an object.

        Parameters:
            dSubsystem: The drive subsystem

            brain: The Vex V5 brain
        '''
        self.apriltags = [                                                      # Defines all of the AprilTags used for the towers
            Tagdesc(0),                                                         #  | Tag 0
            Tagdesc(1),                                                         #  | Tag 1
            Tagdesc(2),                                                         #  | Tag 2
            Tagdesc(3),                                                         #  | Tag 3
        ]                                                                       # Ends the list
        
        self.colors = [                                                         # Defines the colors to detect
            Colordesc(1, 71, 200, 200, 20, 0.4)                                 #  | The cyan color of the paper height tags (May require updating after changing lighting)
        ]                                                                       # Ends the list

        self.aiVisionSensor = AiVision(Ports.PORT10, self.colors[0])            # Initializes the AI Vision Sensor
        self.aiVisionSensor.start_awb()                                         # Starts auto white balance
        self.aiVisionSensor.tag_detection(True)                                 # Enables AprilTag detection
        self.aiVisionSensor.model_detection(False)                              # Ensures AI model detection is disabled
        self.aiVisionSensor.color_detection(True, True)                         # Enables color detection

        self.tagSnapshots = [                                                   # Defines the list of tag snapshots
            self.aiVisionSensor.take_snapshot(self.apriltags[0]),               #  | Snapshot of tag 0
            self.aiVisionSensor.take_snapshot(self.apriltags[1]),               #  | Snapshot of tag 1
            self.aiVisionSensor.take_snapshot(self.apriltags[2]),               #  | Snapshot of tag 2
            self.aiVisionSensor.take_snapshot(self.apriltags[3]),               #  | Snapshot of tag 3
        ]                                                                       # Ends the list

        self.colorSnapshots = [                                                 # Defines the list of color snapshots
            self.aiVisionSensor.take_snapshot(self.colors[0])                   #  | Snapshot of the cyan height tags
        ]                                                                       # Ends the list

        self.driveSubsystem = dSubsystem                                        # Makes the drive subsystem passed to this object available to the rest of the object
        self.brain = brain                                                      # Makes the brain varaible passed to this object available to the rest of the object

        self.scanning = True                                                    # Enables scanning by default
        self.tagHeadings = [                                                    # Creates a list of tag headings
            0.0,                                                                #  |  
            0.0,                                                                #   > Sets all headings to zero to update later
            0.0,                                                                #  |
            0.0                                                                 #  |
        ]                                                                       # Ends the list

        self.tagHeadingLists = [                                                # Defines the lists used to create the above headings
            [],                                                                 #  |
            [],                                                                 #   > Create blank lists to be filled later
            [],                                                                 #  |
            []                                                                  #  |
        ]                                                                       # Ends the list

    def periodic(self):
        '''
        Runs every twenty milliseconds.

        Used for updating the tag snapshots, scanning, and getting the height of box towers
        '''
        if self.aiVisionSensor.installed():                                                     # If the AI Vision Sensor is connected, run below
            for i in range(len(self.tagSnapshots)):                                             # Loop for the length of the tagSnapshots list
                self.tagSnapshots[i] = self.aiVisionSensor.take_snapshot(self.apriltags[i])     # Update each snapshot for its respective tag
                
                if self.scanning:                                                               # If scanning for tag headings, run following
                    for object in self.tagSnapshots[i]:                                         # Loop for each detected object in the snapshot
                        if object.id is not None and 150 < object.centerX < 170:                # If a tag is detected and near the center of the camera
                            self.tagHeadingLists[i].append(self.driveSubsystem.heading-10)      # Add the current robot heading to the respective tag heading list
                                                                                                                            #  /\ (with a little correction)
            
            for i in range(len(self.colorSnapshots)):                                           # Loop for each color that need to be detected
                self.colorSnapshots[i] = self.aiVisionSensor.take_snapshot(self.colors[i])      # Take a snapshot of the current color

            self.towerHeight = len(self.colorSnapshots[0])                                      # count the number of height tags detected and set the tower height
    
    def averageHeadings(self):
        '''
        Averages the headings stored in tagHeadingsLists into the actual tag headings
        '''
        for i in range(len(self.tagHeadingLists)):                              # Loop through the lists of tag headings
            self.tagHeadings[i] = 0                                             # Set the tag heading to zero
            for data in self.tagHeadingLists[i]:                                # Loop through data in the current heading list
                self.tagHeadings[i] += data                                     # Add the data to the heading
            if len(self.tagHeadingLists[i]) > 0:
                self.tagHeadings[i] /= len(self.tagHeadingLists[i])

        self.tagHeadingLists = [                                                # Reset the tag heading lists back to zero
            [],                                                                 #  |
            [],                                                                 #  |
            [],                                                                 #  |
            []                                                                  #  |
        ]                                                                       # End of list

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
    dSubsystem.turnToHeading(vSubsystem.tagHeadings[id])
    while dSubsystem.turningToTarget: wait(20, MSEC)
    wait(250, MSEC)
    proximity = 100
    if vSubsystem.towerHeight == 0: cSubsystem.setArmHeight(ArmHeights.FLOOR); proximity = 30
    elif vSubsystem.towerHeight == 1: cSubsystem.setArmHeight(ArmHeights.LEVEL_1); proximity = 25
    elif vSubsystem.towerHeight == 2: cSubsystem.setArmHeight(ArmHeights.LEVEL_2); proximity = 25
    elif vSubsystem.towerHeight == 3: cSubsystem.setArmHeight(ArmHeights.LEVEL_3); proximity = 30
    wait(50, MSEC)
    while not cSubsystem.armAtTarget: wait(20, MSEC)
    dSubsystem.driveRobotOriented(50, 0)
    while vSubsystem.tagSnapshots[id][0].width < proximity:
        if not vSubsystem.tagHeadings[id]-1.5 < dSubsystem.heading < vSubsystem.tagHeadings[id]+1.5:
            dSubsystem.turnToHeading(vSubsystem.tagHeadings[id])
        wait(20, MSEC)
    dSubsystem.stopDrive()
    cSubsystem.openClaw()
    while cSubsystem.currentClawState == ClawStates.CLOSED: wait(20, MSEC)
    dSubsystem.driveRobotOriented(-50, 0)
    wait(1, SECONDS)
    cSubsystem.setArmHeight(ArmHeights.FLOOR)
    dSubsystem.stopDrive()

def ScanForTags(toZero: bool, dSubsystem: DriveSubsystem, vSubsystem: VisionSubsystem):
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


brain = Brain()                                                                 # defines the Vex V5 brain
controller = Controller()                                                       # defines the Vex V5 controller

driveSubsystem = DriveSubsystem(brain)                                          # defines the drive subsystem
visionSubsystem = VisionSubsystem(driveSubsystem, brain)                        # defines the vision subsystem
clawSubsystem = ClawSubsystem(brain)                                            # defines the claw subsystem

emergencyStopEvent = controller.buttonLeft.pressed(brain.program_stop)          # ends the program when you hit the left arrow on the controller
controller.buttonR1.pressed(clawSubsystem.raiseArm)
controller.buttonR2.pressed(clawSubsystem.lowerArm)


def periodicThread():
    '''
    All of the code in here is used for updating the various subsystems
    '''
    while True:                                                                 # Loop the following code forever
        driveSubsystem.periodic()                                               # runs the drive subsystem's periodic function 
        visionSubsystem.periodic()                                              # runs the vision subsystem's periodic function
        clawSubsystem.periodic()                                                # runs the claw subsystem's periodic function
        displayDebugData()
        wait(20, MSEC)                                                          # delays the thread twenty milliseconds so it doesn't take up all of the resources

def displayDebugData():
    controller.screen.clear_screen()
    controller.screen.set_cursor(1, 1)
    controller.screen.print(clawSubsystem.targetArmHeight)
    controller.screen.next_row()
    controller.screen.print(clawSubsystem.armMotor.position())
    controller.screen.next_row()
    controller.screen.print(str(clawSubsystem.armAtTarget))



if __name__ == "__main__": 
    wait(2, SECONDS)                                                            # Delays the program two seconds to allow the gyro to fully calibrate

    periodic = Thread(periodicThread)                                           # Initializes the periodic thread so it will run seperately from the main code

    ScanForTags(False, driveSubsystem, visionSubsystem)                         # Scans for tags without turning to zero first

    CollectFromStack(0, driveSubsystem, visionSubsystem, clawSubsystem)         # Collects from the stack with AprilTag 0

    ScanForTags(True, driveSubsystem, visionSubsystem)                          # Scans for tags at the new location, turning to zero first

    DropInStack(1, driveSubsystem, visionSubsystem, clawSubsystem)              # Drops the ball in the stack with AprilTag 1

    ScanForTags(True, driveSubsystem, visionSubsystem)                          # Scans for tags at the new location, turning to zero first

    CollectFromStack(2, driveSubsystem, visionSubsystem, clawSubsystem)         # Collects from the stack with AprilTag 2

    ScanForTags(True, driveSubsystem, visionSubsystem)                          # Scans for tags at the new location, turning to zero first

    DropInStack(3, driveSubsystem, visionSubsystem, clawSubsystem)              # Drops the ball in the stack with AprilTag 3