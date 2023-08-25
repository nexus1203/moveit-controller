#!/usr/bin/env python3

import rospy
from robot_controller import Robot
from robot_controller import PoseConverter, calculate_relative_pose



def wait(on=True):
    """Waits for user input to continue

    Args:
        on (bool, optional): enable or disable waiting. Defaults to True.
    """
    if on:
        inp = input('continue? Y/N')
        if inp == "N":
            return exit()


def sleep(t):
    """Sleeps for t seconds using rospy.sleep

    Args:
        t (float): time in seconds
    """
    duration = rospy.Duration(t)
    rospy.sleep(duration)



if __name__ == '__main__':
    rospy.init_node('robot_control', anonymous=True)
    sleep(1)
    robot = Robot()
    
    # enable robot if working with real robot
    robot.enable()
    
    # define target pose
    pose = PoseConverter([0.5, 0.5, 0.5], [0, 0, 0]).get_pose()
    
    # move robot without blocking using move_ptp
    robot.move_ptp(pose, blocking=False)
    
    # wait for robot to finish movement
    while robot.in_motion():
        sleep(0.1)
        
    # define target pose
    pose = PoseConverter([0.45, 0.45, 0.45], [0, 0, 0]).get_pose()
    
    # move robot while blocking using cartesian movement
    # this line will block until the robot has reached the target pose
    robot.move_cartesian_path([pose], blocking=True)
    
 
    # get current pose
    current_pose = robot.get_tool_pose()
    # calculate relative pose
    new_pose = calculate_relative_pose(current_pose, position=[0, 0, 0.1])
    
    # move using linear movement
    robot.move_linear(new_pose, blocking=True)
    
    
    
    