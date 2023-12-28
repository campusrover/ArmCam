# Arm Cam
The new and improved method for working with the PX-100 arm in the Autonomous Robotics Lab at Brandeis University

### For help

Benjamin Blinder, [benjaminblinder@brandeis.edu](mailto:benjaminblinder@brandeis.edu)

## Summary

The PX-100 Arm was a nightmare to use. This project aims to simplify and standardize the useage of the arm. It assumes that you are still using the large rig setup with a webcam in the green holder, and the arm screwed onto the base. It also assumes you are using a computer that has been properly set up with Ubuntu 20.04.6 and both the arm and camera are connected to the laptop

# How To Use

## Setup

If you are in the lab, make sure to use either of the laptops that are marked with pink tape and say "Works with the arm." 
If you use on of these arms, skip the "Running the Code" Section.

If you want to setup a new laptop for use with the arm, here is the a brief overview of the arm setup. It is worth your time to follow the setup instructions for Ubuntu [here](https://ubuntu.com/tutorials/install-ubuntu-desktop?ref=mrdbourke.com#1-overview) and make sure you use Ubuntu 20.04.6, as well as the instructions [here](https://campus-rover.gitbook.io/lab-notebook/campusrover-lab-notebook/faq/interbotixpincherx100) for setting up the Interbotix Workspace.
>[!WARNING]
>I would highly recommend using one of the already working laptops. Some laptops simply do not have compatible hardware with the PX-100 arm and therefore will not work, and you will not know this until you've already spent hours setting up the laptop.
That being said, the issue with hardware compatibility can be fixed by replacing the U2D2 Controller in the arm with a new one from Trossen Robotics. 

1. Install **Ubuntu 20.04.6**
This is the only version that I know for sure works, but other versions of 20.04 might work.

2. Install **Ros Noetic** from [this link](http://wiki.ros.org/noetic/Installation/Ubuntu)

3. Install pip, [these instructions](https://pip.pypa.io/en/stable/installation/#) seem fine but I cannot verify since I did not use it for my installation.

4. Using pip, run this command in terminal:
```
pip install tk
```
5. Install the Interbotix Workspace described [here](https://campus-rover.gitbook.io/lab-notebook/campusrover-lab-notebook/faq/interbotixpincherx100).

6. Clone the [rosutils repo](https://campus-rover.gitbook.io/lab-notebook/campusrover-lab-notebook/bru/rosutils) and follow the new VM instructions.

7. Correctly set up the Bashrc file with the template from the rosutils

8. Change Line 11 in control_arm.py to match the path to the interbotix workspace on your laptop

9. Make sure the arm is plugged into the laptop via one usb port in the laptop **and the arm's power cable is plugged in**.

10. Connect the camera to a second usb port on the laptop.

11. Open a terminal and type the following command to start the program:
```
roslaunch armcam arm_command.launch
```
## Troubleshooting
- The most common issue is the arm not being detected correctly. This is when the RViz window opens but most of the arm is colored bright white and refuses to accept commands. *To fix this issue,* ensure that your interbotix workspace is correctly setup and the path is correctly set in your control_arm.py file. 

    If the arm is still not being recognized after that, you will need to install the Dynamixel software and ensure that it finds the arm and all the motors correctly.

    If that fails, then your computer is most likely incompatible with the arm.

- If the camera is not being picked up, make sure your bashrc file is set up consistently with the rosutils template.
- If you want to use a standard robot along with the arm, then you will need to make a link between the bashrc $path and the bru.py file from rosutils. You might also need to ensure that tailscale is working and connects to the lab network correctly. 

## Using the Arm
**By now, you should have the arm set up and the software capable of running**

The useage of the arm is farily simple. The black tape on the base shows where an object can be reached from.

Run the program using te following command in the terminal:
```
roslaunch armcam arm_command.launch
```

The arm should move to a start location, and a Tkinter window should open (It does not have an icon in the sidebar, but it is labeled "Control Panel")

The controls on the Tkinter window are fairly easy to use. The 3 entry widgets at the top are x,y, and z coordinates for the arm, which are sent with the Send Coords button.

The start button will begin the automatic pickup of an object.

The Sleep button sends the arm to the sleep position.

The Home button sends the arm to the home position.

The red Open and Close gripper buttons open and close the gripper on the arm.

The time scale changes how long it takes for commands to run, and the default is 1.

At the bottom, the Controls button pulls up the standard control window (Which is what starts open) and the Color Test window opens up a live color testing window. 

In the live color testing window, the program pulls up a live view of the binary image that is detected after processing the image. This can be changed to include a wider or narrower range of colors by changing the color arrays.

Color arrays are set in RGB values, the upper array is the brighter value that can be seen, and the lower array is the darker value. These can be adjusted live, **but to save any changes to the color detection, you must change the arrays saved on lines 13 and 14 of color_mask_test.py (Which is only used during the live testing).**
> This will not change the colors used in picking up an object automatically unless you also change the chosen color arrays in the arm_cam.py file, lines 17 to 21.
