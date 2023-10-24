# Cargo Claw

### Members

Benjamin Blinder, [benjaminblinder@brandeis.edu](mailto:benjaminblinder@brandeis.edu)

Ken Kirio, [kenkirio@brandeis.edu](mailto:kenkirio@brandeis.edu)

Vibhu Singh, [vibhusingh@brandeis.edu](mailto:kenkirio@brandeis.edu)

## Summary

Coordinate a stationary arm robot and a platform robot to load and unload cargo autonomously. A person will be able to press a button to inform the robot to go to the arm, which will load/unload a piece of cargo.

## Demonstration

The platform robot will start in a randomized location in the room. After determining its location on a premade map, the robot will maneuver itself to the loading zone, where a person will place an object on top of the robot. A person will press a button on their computer, prompting the robot to leave the loading zone and travel to the arm. Once the robot arrives, the arm will remove the robot's current cargo and place it next to itself, and then place a new piece of cargo onto the platform robot. Then the platform robot will return to its loading zone and again await a person to once again press a button on their computer, prompting the robot to return to the arm robot, which will offload the cargo. This cycle can be repeated as long as someone presses the button to tell it to make another round.

## Learning Objectives

- Mapping: building a map, localization on a premade map
- Image processing & fiducials
- Communication between multiple robots
- Using robots to manipulate other objects
- Coordination with robots not natively made for ROS

## Evaluation Criteria

- Success at picking up/putting down an object with the arm
  - Placing the object in the correct location (i.e. on the platform robot)
- Precision of movement between loading/unloading zones
- Coordination between platform and arm robots so that each one acts at the proper time