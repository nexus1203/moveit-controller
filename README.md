# moveit-controller
A python library for controlling the ROS1 robot using moveit commander interface.

Use this with ROS1 and moveit. You can either control the simulated robot in Rviz or a real robot. 
This library will raise Planning Error if moveit fails to generate a trajectory. You can skip it by applying try/except and try different types of trajectory planning to achieve the motion.
However, for safety purposes on real robot Planning Error was implimented. If you are working with real robot use extreme caution and verify on simulation first before running the code on real robot.

# example usage

```
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

    # disable robot if using real robot
    robot.disable()
```
# Contribution
You can contribute to this respository if you have suggestions for improvements or want to add new features.

# Support
You can open an issue if you need any help with the functionality of this library.

# Disclaimer:
The code is supplied as is. The author takes no responsiblity for safety and effectiveness. The code was tested on both simulated and real robot but if you are using this code to control a real robot, follow all industrial safety guidelines and be responsible.

# Donate
This work is solely done to help the ROS user and new learners to get started with the moveit using the python api. If you think its helpful then please consider supporting the author.

ETH based tokens: ```0xC4deD24f918a89983620654D95dA08CFA63d1628```

Others: contact the author.
