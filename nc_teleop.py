#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import Float64
import sys
import time
import math
import curses
from curses import wrapper
import sys


###################### CURSES PART ##########################
# Prints a line in the center of the specified screen
def print_line_center(message, screen):
    
    # Get the number of rows and columns so that text can be centered in the terminal window.
    num_rows, num_cols = screen.getmaxyx()
    
    # Calculate center row
    middle_row = int(num_rows / 2)
    
    # Calculate center column, and then adjust starting position based
    # on the length of the message
    half_length_of_message = int(len(message) / 2)
    middle_column = int(num_cols / 2)
    x_position = middle_column - half_length_of_message
    
    # Draw the text
    screen.addstr(middle_row, x_position, message)

# Prints a line in the center of the specified screen on the bottom
def print_line_bottom_center(message, screen, offset=0):
    
    # Get the number of rows and columns so that text can be centered in the terminal window.
    num_rows, num_cols = screen.getmaxyx()
    
    # Calculate center row
    middle_row = int(num_rows / 2 + num_rows / 4 + offset)
    
    # Calculate center column, and then adjust starting position based
    # on the length of the message
    half_length_of_message = int(len(message) / 2)
    middle_column = int(num_cols / 2)
    x_position = middle_column - half_length_of_message
    
    # Draw the text
    screen.addstr(middle_row, x_position, message)

#Prints a line in the center of the specified screen on the top
def print_line_top_center(message, screen, offset=0):
    
    # Get the number of rows and columns so that text can be centered in the terminal window.
    num_rows, num_cols = screen.getmaxyx()
    
    # Calculate center row
    middle_row = int(num_rows/4 + offset)
    
    # Calculate center column, and then adjust starting position based
    # on the length of the message
    half_length_of_message = int(len(message) / 2)
    middle_column = int(num_cols / 2)
    x_position = middle_column - half_length_of_message
    
    # Draw the text
    screen.addstr(middle_row, x_position, message)

# Prints the driving keys at the center of the specified screen
def print_keys(key, screen):
    
    # Get the number of rows and columns so that text can be centered in the terminal window.
    num_rows, num_cols = screen.getmaxyx()
    
    keys = "h j k l"
    
    if key == "h":
        
        arrows = "<       "
        
    elif key == "j":
        
        arrows = "  ^     "
        
    elif key == "k":
        
        arrows = "    v   "
        
    elif key == "l":
        
        arrows = "      >"
        
    else:
        
        arrows = "< ^ v >"
    
    # Calculate center row
    middle_row = int(num_rows / 2)
    
    # Calculate center column, and then adjust starting position based
    # on the length of the message
    half_length_of_message = int(len(keys) / 2)
    middle_column = int(num_cols / 2)
    x_position = middle_column - half_length_of_message
    
    # Draw the text
    screen.addstr(middle_row+1, x_position, arrows)
    screen.addstr(middle_row, x_position, keys)

# Checks and resizes the window if the terminal has changed size
def check_screen_resize(screen):
    
    if screen.getch() == curses.KEY_RESIZE:
        curses.resizeterm(*screen.getmaxyx())
        screen.clear()
        screen.border()
        screen.refresh()

    
def main(main_screen):
    
    # Turns off the cursor
    curses.curs_set(0)
    
    # Initialize the screen the first time.before the loop starts
    key = "b"
    main_screen.border()
    print_keys(key, main_screen)
    print_line_top_center("(Current setspeed: {0})".format(1), main_screen)
    print_line_bottom_center("Use the above keys to drive the robot.", main_screen)
    print_line_bottom_center("Controls:", main_screen, 2)
    print_line_bottom_center("f - faster", main_screen, 3)
    print_line_bottom_center("s - slower", main_screen, 4)
    print_line_bottom_center("Space - stop", main_screen, 5)
    print_line_bottom_center("q - exit", main_screen, 6)
    main_screen.refresh()
    
    # Speed of the robot
    setpoint = 1
    

    rate = rospy.Rate(100) # 100hz

    front_left = rospy.Publisher('/front_left_wheel_controller/command', Float64, queue_size=1000)
    front_right = rospy.Publisher('/front_right_wheel_controller/command', Float64, queue_size=1000)
    back_left = rospy.Publisher('/back_left_wheel_controller/command', Float64, queue_size=1000)
    back_right = rospy.Publisher('/back_right_wheel_controller/command', Float64, queue_size=1000)
 
    
    while True:
        
        # check_screen_resize(main_screen)
        main_screen.border()
        key = chr(main_screen.getch())
       
        if key == "q":
            front_left.publish(float(0))
            front_right.publish(float(0))
            back_left.publish(float(0))
            back_right.publish(float(0))
            sys.exit()
        elif key == "h":
            front_left.publish(float(-setpoint))
            front_right.publish(float(setpoint))
            back_left.publish(float(-setpoint))
            back_right.publish(float(setpoint))
        elif key == "j":
            front_left.publish(float(-setpoint))
            front_right.publish(float(-setpoint))
            back_left.publish(float(-setpoint))
            back_right.publish(float(-setpoint))
        elif key == "k":
            front_left.publish(float(setpoint))
            front_right.publish(float(setpoint))
            back_left.publish(float(setpoint))
            back_right.publish(float(setpoint))
        elif key == "l":
            front_left.publish(float(setpoint))
            front_right.publish(float(-setpoint))
            back_left.publish(float(setpoint))
            back_right.publish(float(-setpoint))
        elif key == "f":
            setpoint += 0.25
        elif key == "s":
            setpoint -= 0.25
        else:
            front_left.publish(float(0))
            front_right.publish(float(0))
            back_left.publish(float(0))
            back_right.publish(float(0))
        
        main_screen.clear()
        print_keys(key, main_screen)
        
        
        #time.sleep(0.01)
        rate.sleep()
        
        
        
        print_line_top_center("(Current setspeed: {0})".format(setpoint), main_screen)
        print_line_bottom_center("Use the above keys to drive the robot.", main_screen)
        print_line_bottom_center("Controls:", main_screen, 2)
        print_line_bottom_center("f - faster", main_screen, 3)
        print_line_bottom_center("s - slower", main_screen, 4)
        print_line_bottom_center("Space - stop", main_screen, 5)
        print_line_bottom_center("q - exit", main_screen, 6)
        main_screen.refresh()

def movement():
    rospy.init_node('movement', anonymous=True)
    while not rospy.is_shutdown():

        wrapper(main)
        #print("Set the speed of the different sides of the motor")
        #speedL = input("left: ")
        #speedR = input("right: ")
        #rospy.loginfo(float(speedL))



if __name__ == '__main__':
    try:
        movement()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        sys.exit(0)
