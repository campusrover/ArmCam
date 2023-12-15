#!/usr/bin/env python

"""THIS IS THE SAME AS THE STANDARD GUI, BUT WITHOUT ANY ROSPY CALLS SO IT CAN RUN ON ANY COMPUTER"""
"""USE THIS TO GET THE HANG OF TKINTER BEFORE INCLUDING ROSPY FUNCTIONS WITH IT"""

import tkinter #this is the GUI I chose
import subprocess
import time
import os
import signal

"""========================================================================================="""
#Initialize some needed variables
widgets=[] #Holds all tkinter widgets, used to clear the screen

#These hold the 3 numbers inputed by the user
inputX=0
inputY=0
inputZ=0

root=tkinter.Tk() #init window
root.wm_title("Control Panel") #set window name
root.geometry("500x600") #set size of window

"""========================================================================================="""

#All rospy callbacks
message=" "
def ui_cb(msg): #recives string from other nodes
    global message
    message=str(msg.data)
    updateMessages()


def arm_cb(msg): #a callback from data about the arm
    global message
    if (msg.data=="invalid"):
        message = "Invalid Coordinate"
        updateMessages()
    elif (msg.data=="valid"):
        message = "Valid Coordinate"
        updateMessages()

"""========================================================================================="""
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

"""========================================================================================="""
#All needed functions for the GUI

#updates all the text
def updateMessages():
    if message!=" ":
        text1.set(message)
        # wordList=message.split("\n")
        # text1.set(wordList[0])
        # text2.set(wordList[1])
        # text3.set(wordList[2])
        # text4.set(wordList[3])
        # text5.set(wordList[4])
    else:
        text1.set("Please Start Main Node") #If no data is recieved, it tells you to start the main node
        text2.set(" ")
        text3.set(" ")
        text4.set(" ")
        text5.set(" ")

#Packs the standard control window onto the window
def pack_test():
    killOpenTerminal() #Close any open terminals created by the UI
    for i in widgets: #removes all current widgets from the window
        i.pack_forget()
    mode_text.set("Arm Control") #changes the name of the window

    mode_label.pack(side=tkinter.TOP)

    #creating labels to display data
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
    
    
    perma_frame.pack(side=tkinter.TOP,expand=False,fill=tkinter.X)
    go_to_test.pack(side=tkinter.LEFT,expand=True,fill=tkinter.X)
    go_to_color_test.pack(side=tkinter.LEFT,expand=True,fill=tkinter.X)
    go_to_tuner.pack(side=tkinter.LEFT,expand=True,fill=tkinter.X)


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
    time_entry.delete(0,500) #Clears the text entry 
    time_entry.insert(0,"1") #Adds a 1 to it
    time_button.pack(side=tkinter.LEFT, expand=True, fill=tkinter.X)

#Clears the window and then loads the window for color testing
def pack_color_test():
    killOpenTerminal()
    openTestCamera() #Runs the command to open a seperate terminal to run a different node
    for i in widgets:
        i.pack_forget()
    mode_text.set("Color Mask Tester")

    mode_label.pack(side=tkinter.TOP)

    color_label1.pack(side=tkinter.TOP,expand=False,fill=tkinter.X)
    
    color_frame1.pack(side=tkinter.TOP, expand=False, fill=tkinter.BOTH)
    
    color_label2.pack(side=tkinter.TOP,expand=False,fill=tkinter.X)
    
    color_frame2.pack(side=tkinter.TOP, expand=False, fill=tkinter.BOTH)
    color_frame3.pack(side=tkinter.TOP, expand=False, fill=tkinter.BOTH)

    r_entry1.pack(side=tkinter.LEFT, expand=True, fill=tkinter.X)
    g_entry1.pack(side=tkinter.LEFT, expand=True, fill=tkinter.X)
    b_entry1.pack(side=tkinter.LEFT, expand=True, fill=tkinter.X)
    
    r_entry2.pack(side=tkinter.LEFT, expand=True, fill=tkinter.X)
    g_entry2.pack(side=tkinter.LEFT, expand=True, fill=tkinter.X)
    b_entry2.pack(side=tkinter.LEFT, expand=True, fill=tkinter.X)

    color_button.pack(side=tkinter.TOP, expand=True, fill=tkinter.BOTH)


    

    perma_frame.pack(side=tkinter.TOP,expand=False,fill=tkinter.X)

def pack_tuning(): #Mostly empty, a starter example, not used
    killOpenTerminal()
    for i in widgets:
        i.pack_forget()
    mode_text.set("Example Window\n\n")
    pubGoSleep()
    pubOpenGripper()
    

    mode_label.pack(side=tkinter.TOP)


    frame1.pack(side=tkinter.TOP, expand=False, fill=tkinter.X)
    bigframe.pack(side=tkinter.TOP,expand=True, fill=tkinter.BOTH)

    x_entry.pack(side=tkinter.LEFT, expand=False, fill=tkinter.X)
    y_entry.pack(side=tkinter.LEFT, expand=False, fill=tkinter.X)
    z_entry.pack(side=tkinter.LEFT, expand=False, fill=tkinter.X)

    coords_button.pack(side=tkinter.LEFT, expand=True, fill=tkinter.BOTH)


    perma_frame.pack(side=tkinter.TOP,expand=False,fill=tkinter.X)



#All functions that are called by pressing GUI buttons

def pubStartArm():
    print("Start Custom Pickup Function")

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
    print("Send arm to " + str(inputX) +", "+ str(inputY) +", "+ str(inputZ))

def pubGoSleep():
    print("Send arm to the Sleep Position")

def pubGoHome():
    print("Send Arm to the Home Position")

def pubOpenGripper():
    print("Open Gripper")

def pubCloseGripper():
    print("Close Gripper")

def pubSetTime():
    print("Set time to :"+time_entry.get())

def pubSetColor():
    color_string=""
    color_string+=r_entry1.get()+","+g_entry1.get()+","+b_entry1.get()+"-"+r_entry2.get()+","+g_entry2.get()+","+b_entry2.get()
    print("Color Range Set To: "+ color_string)


#terminal commands
process=subprocess
def killOpenTerminal(): #This allows the node to close any terminals it opened
    global process
    try:
        os.killpg(os.getpgid(process.pid),signal.SIGTERM)
    except:
        pass

def openTestCamera():#Runs the command as a subprocess in a way that it can later be ended
    global process
    # command="rosrun armcam color_mask_test.py"
    # process=subprocess.Popen(command, shell=True,preexec_fn=os.setsid)


"""========================================================================================="""
#Making all widgets

mode_label=tkinter.Label(root,textvariable=mode_text, width=500)
label1=tkinter.Label(root,textvariable=text1, width=500)
label2=tkinter.Label(root,textvariable=text2, width=500)
label3=tkinter.Label(root,textvariable=text3, width=500)
label4=tkinter.Label(root,textvariable=text4, width=500)
label5=tkinter.Label(root,textvariable=text5, width=500)

#Permanent buttons at the bottom of the screen
perma_frame=tkinter.Frame(
    root
)
go_to_test=tkinter.Button(
    perma_frame,
    text="Controls",
    bg="grey",
    command=pack_test
)
go_to_color_test=tkinter.Button(
    perma_frame,
    text="Color Test",
    bg="grey",
    command=pack_color_test
)
go_to_tuner=tkinter.Button(
    perma_frame,
    text="Tuning",
    bg="grey",
    command=pack_tuning
)

#Frames to organize the standard window
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

#Frames to organize the color testing window
color_frame1=tkinter.Frame(
    root
)
color_label1=tkinter.Label(root,text="\nColor Upper Bound (RGB)", width=500)

color_frame2=tkinter.Frame(
    root
)
color_label2=tkinter.Label(root,text="\n\nColor Lower Bound (RGB)", width=500)

color_frame3=tkinter.Frame(
    root
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
    bg="green",
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
    bg="#FF4500",
    command=pubOpenGripper
)

close_gripper_button=tkinter.Button(
    frame3,
    text="Close Gripper",
    bg="#FF4500",
    command=pubCloseGripper
)

time_entry=tkinter.Entry(
    frame4,
    justify=tkinter.CENTER
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

#Widgets for the Color Tuning Window
r_entry1=tkinter.Entry(
    color_frame1,
    width=10
)
g_entry1=tkinter.Entry(
    color_frame1,
    width=10
)
b_entry1=tkinter.Entry(
    color_frame1,
    width=10
)
r_entry2=tkinter.Entry(
    color_frame2,
    width=10
)
g_entry2=tkinter.Entry(
    color_frame2,
    width=10
)
b_entry2=tkinter.Entry(
    color_frame2,
    width=10
)

color_button=tkinter.Button(
    color_frame3,
    text="Set Color Arrays",
    bg="light green",
    command=pubSetColor
)


"""========================================================================================="""


"""
In order to clear a window, you need to reference all the widgets that could be on that window
I was lazy and just made one massive list that the program runs through to hit every possible widget
"""
widgets.append(mode_label)
widgets.append(label1)
widgets.append(label2)
widgets.append(label3)
widgets.append(label4)
widgets.append(label5)
widgets.append(x_entry)
widgets.append(y_entry)
widgets.append(z_entry)
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
widgets.append(color_frame1)
widgets.append(color_frame2)
widgets.append(color_frame3)
widgets.append(color_label1)
widgets.append(color_label2)
widgets.append(r_entry1)
widgets.append(r_entry2)
widgets.append(g_entry1)
widgets.append(g_entry2)
widgets.append(b_entry1)
widgets.append(b_entry2)
widgets.append(perma_frame)
widgets.append(time_entry)
widgets.append(time_button)
widgets.append(frame4)
widgets.append(color_button)

"""========================================================================================="""
pack_test()#loads the standard control window first


root.mainloop() #tktiner mainloop, needed to render and upate the window with any changes

#Anything below this will only be run after the window is closed

killOpenTerminal()#Closes open terminals created by this node