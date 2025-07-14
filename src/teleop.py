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
    LEVEL_1 = 252                                                               # One box
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

        Parameters:
            brain: The Vex V5 Brain object
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
        Runs every twenty milliseconds. Used for updating the robot's heading in the code and for controlling the motors.

        Parameters:
            None
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
        
        Parameters:
            speed: The speed to turn the robot at
        '''
        self.turnSpeed = speed                                                  # Changes the turnSpeed variable to match the input speed

    def stopTurn(self):
        '''
        Stops the robot's turn.

        Parameters:
            None
        ''' 
        self.turnSpeed = 0                                                      # Sets the turnSpeed variable to 0

    def driveFieldOriented(self, vertSpeed: int, horSpeed: int):
        '''
        Commands the robot to drive field-oriented, which means that one direction is always forward,
        no matter which way the robot is facing.
        
        Parameters:
            vertSpeed: The speed the robot should move forward and backward relative the original direction
            horSpeed: The speed the robot should move left and right relative the original direction
        '''
        headingRadians = math.radians(self.heading)                             # Gets the robot's current heading in radians
        sinH = math.sin(headingRadians)                                         # Gets the sine of the robot's heading
        cosH = math.cos(headingRadians)                                         # Gets the cosine of the robot's heading
        self.yMoveSpeed = ((cosH * vertSpeed) + (sinH * horSpeed))              # Sets the y movement speed from multiplication of the cosine, sine, and input variables
        self.xMoveSpeed = (-(sinH * vertSpeed) + (cosH * horSpeed))             # Sets the x movement speed from multiplication of the sine, cosine, and input variables
    
    def driveRobotOriented(self, vertSpeed: int, horSpeed: int):
        '''
        Commands the robot to drive oriented to the direction it is facing.

        Parameters:
            vertSpeed: The speed the robot should move forward and backward
            horSpeed: The speed the robot should move left and right
        '''
        self.yMoveSpeed = vertSpeed                                             # Sets the y movement speed to the supplied vertical speed
        self.xMoveSpeed = horSpeed                                              # Sets the x movement speed to the supplied horizontal speed
    
    def stopDrive(self):
        '''
        Stops the robot's driving.

        Parameters:
            None
        '''
        self.yMoveSpeed = 0                                                     # Sets the y movement speed to 0
        self.xMoveSpeed = 0                                                     # Sets the x movement speed to 0

    def turnToHeading(self, target: float):
        '''
        Commands the robot to face a specified heading.

        Parameters:
            target: The target heading
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
            Colordesc(1, 135, 13, 54, 13, 1)                                    #  | The red color of the paper height tags (May require updating after changing lighting)
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
        Runs every twenty milliseconds. Used for updating the tag snapshots, scanning, and getting the height of box towers.

        Parameters:
            None
        '''
        if self.aiVisionSensor.installed():                                                     # If the AI Vision Sensor is connected, run below
            for i in range(len(self.tagSnapshots)):                                             # Loop for the length of the tagSnapshots list
                self.tagSnapshots[i] = self.aiVisionSensor.take_snapshot(self.apriltags[i])     # Update each snapshot for its respective tag
                
                if self.scanning:                                                               # If scanning for tag headings, run following
                    for object in self.tagSnapshots[i]:                                         # Loop for each detected object in the snapshot
                        if object.id is not None and 150 < object.centerX < 170:                # If a tag is detected and near the center of the camera
                            self.tagHeadingLists[i].append(self.driveSubsystem.heading-7.5)     # Add the current robot heading to the respective tag heading list
                                                                                                                            #  /\ (with a little correction)
            
            for i in range(len(self.colorSnapshots)):                                           # Loop for each color that need to be detected
                self.colorSnapshots[i] = self.aiVisionSensor.take_snapshot(self.colors[i])      # Take a snapshot of the current color

            self.towerHeight = len(self.colorSnapshots[0])                                      # count the number of height tags detected and set the tower height
    
    def averageHeadings(self):
        '''
        Averages the headings stored in tagHeadingsLists into the actual tag headings.

        Parameters:
            None
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
    '''
    A subsystem for controlling the claw.
    '''
    def __init__(self, brain: Brain):
        '''
        Initializes the class as an object.

        Parameters:
            brain: The Vex V5 brain
        '''
        self.brain = brain                                                      # Makes the brain argument passed to the object available everywhere in the object
        self.clawMotor = Motor(Ports.PORT3)                                     # Initalizes the claw motor
        self.armMotor = Motor(Ports.PORT8, True)                                # Initializes the arm motor

        self.clawMotor.set_stopping(HOLD)                                       # Sets the claw motor the hold its position when stopped
        self.armMotor.set_stopping(HOLD)                                        # Sets the arm motor the hold its position when stopped

        self.targetClawState = ClawStates.OPEN                                  # Sets the claw's target state to open
        self.currentClawState = ClawStates.OPEN                                 # Sets the claw's current state to open

        self.targetArmHeight = ArmHeights.FLOOR                                 # Sets the target arm height to the floor
        self.armAtTarget = True                                                 # Says the arm is at its target

    def periodic(self):
        '''
        Runs every twenty milliseconds. Used for controlling the motors.

        Parameters:
            None
        '''
        clawMotorSpeed = 0.0                                                                        # Defines the claw speed as zero
        armMotorSpeed = 0.0                                                                         # Defines the arm speed as zero

        if self.targetClawState is ClawStates.OPEN and self.currentClawState is ClawStates.CLOSED:  # If the claw is closed but the target is open
            clawMotorSpeed = -50                                                                    # Run claw at 1/2 speed in reverse
        if self.targetClawState is ClawStates.CLOSED and self.currentClawState is ClawStates.OPEN:  # If the claw is open but the target is closed
            clawMotorSpeed = 50                                                                     # Run claw at half speed forward
        
        if self.clawMotor.position() > 200 and clawMotorSpeed > 0:                                  # If the claw motor is past 200 dg. and running farther
            self.currentClawState = self.targetClawState                                            # Set the current state to the target state
        if self.clawMotor.position() < 10 and clawMotorSpeed < 0:                                   # If the claw motor is below 10 dg. and trying to move backwards
            self.currentClawState = self.targetClawState                                            # Set the current state to the target state

        if self.targetArmHeight-7.5 < self.armMotor.position() < self.targetArmHeight+7.5:          # If the armMotor is within 7.5 dg. of the target
            self.armAtTarget = True                                                                 # Set the armAtTarget variable to True
        else:                                                                                       # Otherwise
            self.armAtTarget = False                                                                # Set the armAtTarget variable to False
        
        if not self.armAtTarget:                                                                    # If the arm isn't at the target
            armMotorSpeed = math.copysign(50, self.targetArmHeight - self.armMotor.position())      # Set the arm motor speed to 50 with the correct sign

        if abs(self.armMotor.position()-self.targetArmHeight) <= 75:                                # If the arm motor is less than 75 dg. off from the target
            armMotorSpeed = armMotorSpeed/7                                                         # Slow the motor to a seventh of the old speed
        elif abs(self.armMotor.position()-self.targetArmHeight) <= 40:                              # If the arm motor is less than 40 dg. off from the target
            armMotorSpeed = armMotorSpeed/9                                                         # Slow the motor to a ninth of the old speed

        if self.armMotor.position() < 0 and armMotorSpeed < 0:                                      # If the arm motor is below zero and trying to move down
            armMotorSpeed = 0.0                                                                     # Set the speed to zero
        if self.armMotor.position() > 700 and armMotorSpeed > 0:                                    # If the arm motor is above 700 and trying to move up
            armMotorSpeed = 0.0                                                                     # Set the speed to zero
        
        self.clawMotor.spin(FORWARD, clawMotorSpeed, VelocityUnits.PERCENT)                         # Spin the claw motor with the specified speed
        self.armMotor.spin(FORWARD, armMotorSpeed, VelocityUnits.PERCENT)                           # Spin the arm motor with the specified speed


    def openClaw(self):
        '''
        Opens the claw.

        Parameters:
            None
        '''
        self.targetClawState = ClawStates.OPEN                                  # Sets the target claw state to open

    def closeClaw(self):
        '''
        Closes the claw.
        
        Parameters:
            None
        '''
        self.targetClawState = ClawStates.CLOSED                                # Sets the target claw state to closed
    
    def toggleClaw(self):
        '''
        Toggles the claw open and closed.
        
        Parameters:
            None
        '''
        if self.targetClawState == ClawStates.OPEN:                             # If the target claw state is open
            self.targetClawState = ClawStates.CLOSED                            # Set the target claw state to closed
        elif self.targetClawState == ClawStates.CLOSED:                         # If the target claw state is closed
            self.targetClawState = ClawStates.OPEN                              # Set the target claw state to open

    def raiseArm(self):
        '''
        Raises the arm one level.
        
        Parameters:
            None
        '''
        if self.targetArmHeight == ArmHeights.FLOOR:                            # If the target arm height is the floor
            self.targetArmHeight = ArmHeights.LEVEL_1                           # raise it to level 1

        elif self.targetArmHeight == ArmHeights.LEVEL_1:                        # Else if the target arm height is level 1
            self.targetArmHeight = ArmHeights.LEVEL_2                           # Raise it to level 2

        elif self.targetArmHeight == ArmHeights.LEVEL_2:                        # Else if the target arm height is level 2
            self.targetArmHeight = ArmHeights.LEVEL_3                           # Raise it to level 3
    
    def lowerArm(self):
        '''
        Lowers the arm one level.
        
        Parameters:
            None
        '''
        if self.targetArmHeight == ArmHeights.LEVEL_3:                          # If the target arm height is level 3
            self.targetArmHeight = ArmHeights.LEVEL_2                           # Lower it to level 2

        elif self.targetArmHeight == ArmHeights.LEVEL_2:                        # If the target arm height is level 2
            self.targetArmHeight = ArmHeights.LEVEL_1                           # Set it to level 1

        elif self.targetArmHeight == ArmHeights.LEVEL_1:                        # If the target arm height is level 1
            self.targetArmHeight = ArmHeights.FLOOR                             # Set it to the floor
    
    def setArmHeight(self, height: int):
        '''
        Set the arm to the specified height
        
        Parameters:
            height: The height the arm should go to
        '''
        self.targetArmHeight = height                                           # Sets the target arm height to the specified height

#endregion Subsystems

#region Commands

def DriveToTag(id, speed, proximity, dSubsystem: DriveSubsystem, vSubsystem: VisionSubsystem):
    '''
    Commands the robot to face a specified apriltag and then drive to with with the specified proximity.

    Parameters:
        id: The id of the AprilTag to drive to
        speed: The speed to approach the AprilTag at
        proximity: the distance from the AprilTag to drive to (less == farther away)
        dSubsystem: the drive subsystem
        vSubsystem: the vision subsystem
    '''
    dSubsystem.turnToHeading(vSubsystem.tagHeadings[id])                        # Turn to face the AprilTag
    while dSubsystem.turningToTarget: pass                                      # Wait until the turning is over

    dSubsystem.driveRobotOriented(speed, 0)                                     # Drive forward at the passed speed
    while vSubsystem.tagSnapshots[1][0].width < proximity: pass                 # Wait until at the passed proximity

    dSubsystem.stopDrive()                                                      # Stop driving

def CollectFromStack(id, dSubsystem: DriveSubsystem, vSubsystem: VisionSubsystem, cSubsystem: ClawSubsystem):
    '''
    Commands the robot to collect a ball from the specified stack.

    Parameters:
        id: The id of the AprilTag to drive to
        dSubsystem: the drive subsystem
        vSubsystem: the vision subsystem
        cSubsystem: the claw subsystem
    '''
    dSubsystem.turnToHeading(vSubsystem.tagHeadings[id])                        # Turn to face the specified AprilTag
    while dSubsystem.turningToTarget: wait(20, MSEC)                            # Wait until turning is complete

    wait(250, MSEC)                                                             # Wait an additional 1/4 second to allow the camera to find the height tags

    proximity = 100                                                                                 # Set proximity high (it will get changed to be lower)
    if vSubsystem.towerHeight == 0: cSubsystem.setArmHeight(ArmHeights.FLOOR); proximity = 30       # If the tower height is 0, set the arm to the floor and the proximity to 30
    elif vSubsystem.towerHeight == 1: cSubsystem.setArmHeight(ArmHeights.LEVEL_1); proximity = 25   # If the tower height is 1, set the arm to level 1 and the proximity to 25
    elif vSubsystem.towerHeight == 2: cSubsystem.setArmHeight(ArmHeights.LEVEL_2); proximity = 23   # If the tower height is 2, set the arm to level 2 and the proximity to 23
    elif vSubsystem.towerHeight == 3: cSubsystem.setArmHeight(ArmHeights.LEVEL_3); proximity = 30   # If the tower height is 3, set the arm to level 3 and the proximity to 30

    wait(50, MSEC)                                                              # Wait 50 milliseconds to give the claw subsystem time to process the new target arm height
    while not cSubsystem.armAtTarget: pass                                      # wait until the arm is at its target

    dSubsystem.driveRobotOriented(50, 0)                                        # Drive forward at half power
    while vSubsystem.tagSnapshots[id][0].width < proximity: pass                # Wait until the robot is within the specified proximity
    dSubsystem.stopDrive()                                                      # Stop driving
    
    cSubsystem.closeClaw()                                                      # Close the claw
    while cSubsystem.currentClawState == ClawStates.OPEN: pass                  # Wait for the claw to e closed

    dSubsystem.driveRobotOriented(-50, 0)                                       # Drive backward at half power
    wait(1.5, SECONDS)                                                          # Wait 1 1/2 seconds

    cSubsystem.setArmHeight(ArmHeights.FLOOR)                                   # Start lowering the claw to the floor

    dSubsystem.stopDrive()                                                      # Stop driving

def DropInStack(id, dSubsystem: DriveSubsystem, vSubsystem: VisionSubsystem, cSubsystem: ClawSubsystem):
    '''
    Commands the robot to dop a ball into the specified stack.

    Parameters:
        id: The id of the AprilTag to drive to
        dSubsystem: the drive subsystem
        vSubsystem: the vision subsystem
        cSubsystem: the claw subsystem
    '''
    dSubsystem.turnToHeading(vSubsystem.tagHeadings[id])                        # Turn to face the specified AprilTag
    while dSubsystem.turningToTarget: wait(20, MSEC)                            # Wait until turning is complete

    wait(250, MSEC)                                                             # Wait an additional 1/4 second to allow the camera to find the height tags

    proximity = 100                                                                                     # Set proximity high (it will get changed to be lower)
    if vSubsystem.towerHeight == 0: cSubsystem.setArmHeight(ArmHeights.FLOOR); proximity = 30           # If the tower height is 0, set the arm to the floor and the proximity to 30
    elif vSubsystem.towerHeight == 1: cSubsystem.setArmHeight(ArmHeights.LEVEL_1+20); proximity = 25    # If the tower height is 1, set the arm to level 1 and the proximity to 25
    elif vSubsystem.towerHeight == 2: cSubsystem.setArmHeight(ArmHeights.LEVEL_2+20); proximity = 25    # If the tower height is 2, set the arm to level 2 and the proximity to 23
    elif vSubsystem.towerHeight == 3: cSubsystem.setArmHeight(ArmHeights.LEVEL_3+10); proximity = 30    # If the tower height is 3, set the arm to level 3 and the proximity to 30

    wait(50, MSEC)                                                              # Wait 50 milliseconds to give the claw subsystem time to process the new target arm height
    while not cSubsystem.armAtTarget: wait(20, MSEC)                            # wait until the arm is at its target

    dSubsystem.driveRobotOriented(50, 0)                                        # Drive forward at half power 

    while vSubsystem.tagSnapshots[id][0].width < proximity:                     # While outside of the determined proximity, run below
        if not vSubsystem.tagHeadings[id]-1.5 < dSubsystem.heading < vSubsystem.tagHeadings[id]+1.5:    # If not close to the target heading
            dSubsystem.turnToHeading(vSubsystem.tagHeadings[id])                # Turn to the heading
        wait(20, MSEC)                                                          # Wait 20 milliseconds

    dSubsystem.stopDrive()                                                      # Stop driving

    cSubsystem.openClaw()                                                       # Open the claw
    while cSubsystem.currentClawState == ClawStates.CLOSED: wait(20, MSEC)      # Wait until the claw is open

    dSubsystem.driveRobotOriented(-50, 0)                                       # Drive backward at half power
    wait(1.5, SECONDS)                                                          # Wait 1 1/2 seconds

    cSubsystem.setArmHeight(ArmHeights.FLOOR)                                   # Set the target arm height back to the floor

    dSubsystem.stopDrive()                                                      # Stop driving

def ScanForTags(toZero: bool, dSubsystem: DriveSubsystem, vSubsystem: VisionSubsystem):
    '''
    Updates the vision subsystem's list of tag headings.
    Parameters:
        toZero: Determines if the robot should turn to zero heading first
        dSubsystem: the drive subsystem
        vSubsystem: the vision subsystem
    '''
    if toZero:                                                                  # If directed to turn back to zero
        dSubsystem.turnToHeading(5)                                             # turn to five dg.
        while dSubsystem.turningToTarget: pass                                  # Wait until done turning
        wait(100, MSEC)                                                         # Wait a tenth of a second

    dSubsystem.turn(-10)                                                        # Start turning at a tenth of the max turning speed
    vSubsystem.scanning = True                                                  # Start scanning

    steps = 0                                                                   # Set the steps completed to zero
    while True:                                                                 # Loop forever
        steps += 1                                                              # Increase steps by one
        if dSubsystem.heading > 355 and steps > 50:                             # If the robot is above 355 dg. in heading and steps is greater than 50
            vSubsystem.scanning = False                                         # Stop scanning for tags
            dSubsystem.stopTurn()                                               # Stop turning
            vSubsystem.averageHeadings()                                        # Average the headings stored in the lists in the vision subsystem
            break                                                               # break out of the infinite loop
        wait(20, MSEC)                                                          # wait twenty milliseconds

#endregion Commands


brain = Brain()                                                                 # defines the Vex V5 brain
controller = Controller()                                                       # defines the Vex V5 controller

driveSubsystem = DriveSubsystem(brain)                                          # defines the drive subsystem
visionSubsystem = VisionSubsystem(driveSubsystem, brain)                        # defines the vision subsystem
clawSubsystem = ClawSubsystem(brain)                                            # defines the claw subsystem

emergencyStopEvent = controller.buttonLeft.pressed(brain.program_stop)          # ends the program when you hit the left arrow on the controller
controller.buttonR1.pressed(clawSubsystem.raiseArm)
controller.buttonR2.pressed(clawSubsystem.lowerArm)
controller.buttonA.pressed(clawSubsystem.toggleClaw)


def periodicThread():
    '''
    All of the code in here is used for updating the various subsystems
    '''
    while True:                                                                 # Loop the following code forever
        driveSubsystem.periodic()                                               # runs the drive subsystem's periodic function 
        visionSubsystem.periodic()                                              # runs the vision subsystem's periodic function
        clawSubsystem.periodic()                                                # runs the claw subsystem's periodic function
        wait(20, MSEC)                                                          # delays the thread twenty milliseconds so it doesn't take up all of the resources

wait(2, SECONDS)


while True:
    driveSubsystem.driveFieldOriented(controller.axis3.position(), controller.axis4.position())
    driveSubsystem.turn(controller.axis1.position())
    wait(20, MSEC)