#!/usr/bin/env python
import tkinter #this is the GUI I chose
# from std_msgs.msg import String, Bool
# from geometry_msgs.msg import Point
# import rospy
# import rosnode
import subprocess
import time
import os
import signal


widgets=[]
screen1=[]
screen2=[]
screen3=[]

inputX=0
inputY=0
inputZ=0

"""CargobotGUI"""

root=tkinter.Tk() #init window
root.wm_title("Control Panel") #set window name
root.geometry("500x600") #set size of window

message=" "
def ui_cb(msg): #recives string from robot node
    global message
    message=str(msg.data)
    updateMessages()

def state_cb(msg):
    key_pub.publish("gh")

def arm_cb(msg):
    global message
    if (msg.data=="invalid"):
        message = "Invalid Coordinate"
        updateMessages()
    elif (msg.data=="valid"):
        message = "Valid Coordinate"
        updateMessages()

def tkinter_entry_cb(event):
    pass

#creates all the text variables for displaying the robot data
mode_text=tkinter.StringVar()
mode_text.set("Mode 1")

text1=tkinter.StringVar()
text1.set(" ")
text2=tkinter.StringVar()
text2.set(" ")
text3=tkinter.StringVar()
text3.set(" ")
text4=tkinter.StringVar()
text4.set(" ")
text5=tkinter.StringVar()
text5.set(" ")


#updates all the text for robot data
def updateMessages():
    # print(("/robot") in rosnode.get_node_names())
    if message!=" ":
        text1.set(message)
        # wordList=message.split("\n")
        # text1.set(wordList[0])
        # text2.set(wordList[1])
        # text3.set(wordList[2])
        # text4.set(wordList[3])
        # text5.set(wordList[4])
    else:
        text1.set("Please Start Main Node")
        text2.set(" ")
        text3.set(" ")
        text4.set(" ")
        text5.set(" ")

def pack_test():
    for i in widgets:
        i.pack_forget()
    mode_text.set("Arm Control")

    mode_label.pack(side=tkinter.TOP)

    #creating labels to display robot data
    label1.pack(side=tkinter.TOP)
    label2.pack(side=tkinter.TOP)
    label3.pack(side=tkinter.TOP)
    label4.pack(side=tkinter.TOP)
    label5.pack(side=tkinter.TOP)

    #packing frames onto tkinter window
    frame1.pack(side=tkinter.TOP, expand=False, fill=tkinter.X)
    frame2.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)
    frame3.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)
    frame4.pack(side=tkinter.TOP, expand=False, fill=tkinter.BOTH)
    bigframe.pack(side=tkinter.TOP,expand=True, fill=tkinter.BOTH)


    #packing buttons onto the frames in a percise order so it looks right
    x_entry.pack(side=tkinter.LEFT, expand=False, fill=tkinter.X)
    y_entry.pack(side=tkinter.LEFT, expand=False, fill=tkinter.X)
    z_entry.pack(side=tkinter.LEFT, expand=False, fill=tkinter.X)

    coords_button.pack(side=tkinter.LEFT, expand=True, fill=tkinter.BOTH)

    start_button.pack(side=tkinter.LEFT, expand=True, fill=tkinter.BOTH)
    sleep_button.pack(side=tkinter.LEFT, expand=True, fill=tkinter.BOTH)
    home_button.pack(side=tkinter.LEFT, expand=True, fill=tkinter.BOTH)
    open_gripper_button.pack(side=tkinter.LEFT, expand=True, fill=tkinter.BOTH)
    close_gripper_button.pack(side=tkinter.LEFT, expand=True, fill=tkinter.BOTH)

    time_entry.pack(side=tkinter.LEFT, expand=False, fill=tkinter.X)
    time_button.pack(side=tkinter.LEFT, expand=True, fill=tkinter.X)
    # GoGoal.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)
    # back.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)
    # GoHome.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)


#All functions that are called by pressing GUI buttons
def pubStartArm():
    alien_state_publisher.publish(True)

def pubSubmitCoords():
    global inputX,inputY,inputZ,message
    try:
        inputX=float(x_entry.get())
    except:
        inputX=0
    try:
        inputY=float(y_entry.get())
    except:
        inputY=0
    try:
        inputZ=float(z_entry.get())
    except:
        inputZ=0
    point_publisher.publish(Point(inputX, inputY, inputZ))

def pubGoSleep():
    sleep_publisher.publish(True)

def pubGoHome():
    home_publisher.publish(True)

def pubOpenGripper():
    gripper_publisher.publish("open")

def pubCloseGripper():
    gripper_publisher.publish("close")

def pubSetTime():
    try:
        time_float=float(time_entry.get())
    except:
        pass
    time_publisher.publish(time_float)


#initalizing rospy stuff 
# rospy.init_node("GUI")
# key_pub = rospy.Publisher('keys', String, queue_size=1) #publishes to the same topic as key_publisher.py
# ui_sub = rospy.Subscriber('UI', String, ui_cb) #recives the information from the robot node in the UI topic
# arm_subscriber=rospy.Subscriber("arm_status", String, arm_cb)

# alien_state_publisher = rospy.Publisher("alien_state", Bool, queue_size=1)
# point_publisher = rospy.Publisher("/arm_control/point", Point, queue_size=1)
# home_publisher = rospy.Publisher("/arm_control/home", Bool, queue_size=1)
# sleep_publisher = rospy.Publisher("/arm_control/sleep", Bool, queue_size=1)
# gripper_publisher = rospy.Publisher("/arm_control/gripper", String, queue_size=1)
# time_publisher = rospy.Publisher("/arm/time", Float32, queue_size=1) #TODO


mode_label=tkinter.Label(root,textvariable=mode_text, width=500)
label1=tkinter.Label(root,textvariable=text1, width=500)
label2=tkinter.Label(root,textvariable=text2, width=500)
label3=tkinter.Label(root,textvariable=text3, width=500)
label4=tkinter.Label(root,textvariable=text4, width=500)
label5=tkinter.Label(root,textvariable=text5, width=500)





#frames to orgranize buttons
bigframe=tkinter.Frame(
    root
)
frame1=tkinter.Frame(
    bigframe
)
frame2=tkinter.Frame(
    bigframe
)
frame3=tkinter.Frame(
    bigframe
)
frame4=tkinter.Frame(
    bigframe
)


#buttons to press to control the arm
start_button=tkinter.Button(
    frame2,
    text="Start",
    bg="light green",
    command=pubStartArm
)

coords_button=tkinter.Button(
    frame1,
    text="Send Coords (X Y Z)",
    bg="light green",
    command=pubSubmitCoords
)

sleep_button=tkinter.Button(
    frame2,
    text="Sleep",
    bg="light green",
    command=pubGoSleep
)

home_button=tkinter.Button(
    frame2,
    text="Home",
    bg="light green",
    command=pubGoHome
)

open_gripper_button=tkinter.Button(
    frame3,
    text="Open Gripper",
    bg="red",
    command=pubOpenGripper
)

close_gripper_button=tkinter.Button(
    frame3,
    text="Close Gripper",
    bg="grey",
    command=pubCloseGripper
)

time_entry=tkinter.Entry(
    frame4
)

time_button=tkinter.Button(
    frame4,
    text="Set Time Scale",
    bg="grey",
    command=pubSetTime
)

x_entry=tkinter.Entry(
    frame1,
    width=10
)
y_entry=tkinter.Entry(
    frame1,
    width=10
)
z_entry=tkinter.Entry(
    frame1,
    width=10
)


widgets.append(mode_label)
widgets.append(label1)
widgets.append(label2)
widgets.append(label3)
widgets.append(label4)
widgets.append(label5)
widgets.append(x_entry)
widgets.append(y_entry)
widgets.append(z_entry)
for i in widgets:
    screen1.append(i)
    screen2.append(i)
    screen3.append(i)
widgets.append(start_button)
widgets.append(coords_button)
widgets.append(sleep_button)
widgets.append(home_button)
widgets.append(open_gripper_button)
widgets.append(close_gripper_button)
widgets.append(frame1)
widgets.append(frame2)
widgets.append(frame3)
widgets.append(time_button)
widgets.append(bigframe)


pack_test()


root.mainloop() #tktiner mainloop