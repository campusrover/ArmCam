# Arm Cam
The new and improved method for working with the PX-100 arm in the Autonomous Robotics Lab at Brandeis University

### For help

Benjamin Blinder, [benjaminblinder@brandeis.edu](mailto:benjaminblinder@brandeis.edu)

## Summary

The PX-100 Arm was a nightmare to use. This project aims to simplify and standardize the useage of the arm. It assumes that you are still using the large rig setup with a webcam in the green holder, and the arm screwed onto the base. It also assumes you are using a computer that has been properly set up with Ubuntu 20.04.6 and both the arm and camera are connected to the laptop

# How To Use

## Setup

### Step 1: Configuring the Laptop
If you are in the lab, make sure to use either of the laptops that are marked with pink tape and say "Works with the arm." 
If you use on of these arms, skip the "Running the Code" Section.

If you want to setup a new laptop for use with the arm, here is the cliff notes version of the setup:
>[!WARNING]
>I would highly recommend using one of the already working laptops. Some laptops simply do not have compatible hardware with the PX-100 arm and therefore will not work, and you will not know this until you've already spent hours setting up the laptop.
That being said, the issue with hardware compatibility can be fixed by replacing the U2D2 Controller in the arm with a new one from Trossen Robotics. 

1. Install **Ubuntu 20.04.6**
This is the only version that I know for sure works, but other versions of 20.04 might work.

2. Install **Ros Noetic**

3. Install pip

4. Using pip, run this command in terminal:
```
pip install tk
```
5. Install the Interbotix Workspace

6. Clone the rosutils repo

7. Correctly set up the Bashrc file with the template from the rosutils

8. Change Line 11 in control_arm.py to match the path to the interbotix workspace on your laptop


