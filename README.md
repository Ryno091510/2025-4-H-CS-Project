## **Using AprilTags and the AI Vision Sensor for Autonomous Movement in Robots**

*Ryan Mejeur*

*Madison County 4-H Fair - July 2025*



<table>
  <tr>
   <td>
    <img src="README_images/photosQR.png" width="200" alt="alt_text" title="image_tooltip">
   </td>
  </tr>
  <tr>
   <td>
    Scan the QR code above for pictures and videos of the project in action.
   </td>
  </tr>
</table>



## Objective/Goal

The objective of this project is to autonomously control a custom designed and built robot using AprilTags and an AI VIsion Sensor.


## Materials/Equipment



* Vex V5 robotics kit with some additional pieces
    * Base Kit: *www.vexrobotics.com/v5-competition-starter-kit.html*
    * Extras:
        * AI Vision Sensor: *https://www.vexrobotics.com/276-8659.html*
        * Inertial Sensor: *https://www.vexrobotics.com/276-4855.html*
        * 2x V5 Motor: *https://www.vexrobotics.com/276-4840.html*
        * Smart Cables (Short Assortment): *https://www.vexrobotics.com/v5-smart-cables.html*
* AprilTags found here: *https://content.vexrobotics.com/docs/AprilTags_Printable_Letter_REV2.pdf*


## 


## Technical Details



* Programming Language: Python 3

<table>
  <tr>
   <td>
Device
   </td>
   <td>OS
   </td>
   <td>Firmware
   </td>
   <td>Name
   </td>
  </tr>
  <tr>
   <td>Vex V5
   </td>
   <td>vexOS
   </td>
   <td>1.1.5
   </td>
   <td>Ryans_Vex_V5
   </td>
  </tr>
  <tr>
   <td>Programming Computer
   </td>
   <td>Windows 10
   </td>
   <td>10.019045 Build 19045
   </td>
   <td>RYANS-DESKTOP-PC
   </td>
  </tr>
</table>



## User Inputs

There are no user inputs during the actual operation. This is what allows it to be autonomous. The only inputs (done with no human intervention) are the AprilTags and color identifiers on the boxes, which tells the robot where to move and how high to move the arm.

## Setup/Usage

This project will require: Vex robot built with AI Vision Sensor and automated arm, 2 foam baseballs, printed April Tags, multiple boxes with colored pieces of cardstock for identification.



1. Create 4 stacks of boxes either 1, 2, or 3 high with colored tags on each box.
2. Place AprilTags on the bottom box of each stack on the same side as colored tags.
3. Arrange stacks spread apart with tags all facing toward the center.
4. Place the robot in the middle of the stacks.
5. Run the program.

After starting the program, the robot will slowly spin in a circle to scan for AprilTags. After completing one revolution, the robot will turn to face AprilTag 0, whose location was identified during the revolution. It will then raise the claw arm to the appropriate height determined by the colored tags on each box, drive forward, and collect a foam ball from the top of the stack. It will then back out far enough to lower the claw and start another scan. After another revolution and identifying the location of AprilTag 1, it will repeat the process of raising the arm and driving to the stack. This time, instead of picking up a ball, it will drop the one previously collected into the top box. The robot will then back up, lower the claw again, and start the entire sequence over for AprilTags 2 and 3.   


## Build/Coding Process



1. Design and build a robot to accomplish the autonomous tasks
    * Using the VEX V5 Competition Starter Kit with the extra pieces mentioned in the Materials Section, build a robot with a movable claw arm.

         

2. Code a Python program to control the robot in TeleOp mode (user controlled remote)
    * Code the drive subsystem so it can control a holonomic drivetrain. This allows the omni directional wheels to maneuver easily to each stack of boxes.
    * Code the vision subsystem to recognize the AprilTags using the AI vision sensor.
    * Code the claw subsystem to control the opening and closing of the claw and raising and lowering of the arm.
3. Switch the robot to be fully autonomous
    * Connect the 3 subsystems above into the final program, adding the ability to identify the height of the box stacks depending on the number of colored tags it sees.


## Lessons Learned

Based on my other projects completed over the last year, I have learned to break my program down into smaller chunks from the beginning. This made this project much easier to work through.

The sensitivity of the sensors is a lesson I continue to have to learn with each project I build. I underestimated the sensitivity of some of the sensors I was trying to work with. In the end, I removed them and adapted the code instead.


## Debugging/DIfficulties Overcome



1. *Arm stopped moving properly*

    At one point during the build and code process for the arm, the arm stopped moving back down. I had been searching the code for potential reasons but could not find anything. Eventually, I put some debug information onto the remote control screen making the error quick to find and solve. I wasted a lot of time trying to manually find the error.


## Resources



* Vex Clawbot Instructions for only the claw build (not entire robot)
    * *https://content.vexrobotics.com/docs/276-6009-750-Rev6.pdf (P. 16-23)*
* Vex Python Documentation
    * *https://api.vex.com/v5/home/python/index.html*
* Photos
    * 