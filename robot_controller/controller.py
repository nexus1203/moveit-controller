#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from std_srvs.srv import Trigger
import moveit_msgs.msg
from moveit_msgs.msg import MoveItErrorCodes
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander import MoveItCommanderException


from time import sleep
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, cast

from . import io_control
from .pose_utility import PoseConverter, calculate_relative_pose



@dataclass
class Joint:
    name: str
    upper_bound: float
    lower_bound: float
    position: float

    @classmethod
    def from_robot(cls, joint: moveit_commander.RobotCommander.Joint):
        return cls(
            name=joint.name(),
            upper_bound=joint.max_bound(),
            lower_bound=joint.min_bound(),
            position=joint.value(),
        )


class PlanningError(MoveItCommanderException):
    # Map numerical code to some human readable text
    _error_code_map = {
        getattr(MoveItErrorCodes, attr): attr
        for attr in dir(MoveItErrorCodes)
        if attr[0].isalpha() and attr[0].isupper()
    }

    def __init__(self, error_code: int) -> None:
        self.error_code = error_code
        self.message = self._error_code_map[error_code]
        super().__init__(self.message)


class SplinePlanningError(PlanningError):

    def __init__(self, message: str) -> None:
        super().__init__(-1)
        self.message = message


def require_trajectory(func: Callable) -> Callable:

    def inner(self: Any, *args: Any, **kwargs: Any) -> Any:
        if not self._trajectory:  # pylint: disable=protected-access
            raise ValueError("No trajectory, did you forget to plan?")
        return func(self, *args, **kwargs)

    return inner


class Planner:
    DEFAULT = "OMPL"
    PTP = "PTP"
    LIN = "LIN"
    CIRC = "CIRC"
    AnytimeAnytimePath = "AnytimePathShortening"
    BiTRRT = "BiTRRT"
    EST = "EST"
    LBKPIECE1 = "LBKPIECE"
    LazyPRM = "LazyPRM"
    RRT = "RRT"
    FMT = "FMT"
    RRTStar = "RRTstar"
    RRTConnect = "RRTConnect"
    AITStar = "AITstar"
    BITstar = "BITstar"
    SBL = "SBL"
    TRRT = "TRRT"
    ProjEST = "ProjEST"
    SPARS = "SPARS"
    CHOMP = "CHOMP"
    STOMP = 'STOMP'


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(value, max_value))


def check_vel_acc_limits(velocity: float, acceleration: float) -> None:
    return clamp(velocity, 0.001, 1.0), clamp(acceleration, 0.001, 1.0)


class Robot:
    JOINT_ERROR_THRESHOLD: float = 0.001
    JOINT_MEAN_ERROR_THRESHOLD: float = 0.001
    PLANNING_ATTEMPTS: int = 100
    PLANNING_TIME: float = 15.0
    STEP_SIZE: float = 0.01
    OMPL_DEFAULT: str = Planner.RRTStar  # "ompl"

    def __init__(self, group_name="manipulator"):
        """ Initialize the robot controller class and moveit commander

        Args:
            group_name (str, optional): group name of the robot. Defaults to "manipulator".
        """
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(group_name)

        self._reset_state()
        self.tooltip_frame = self.group.get_end_effector_link()
        
        # default planner is PTP
        print("Initial Planner: ", self.group.get_planner_id())
        self.set_planner(Planner.PTP)

    def enable(self) -> None:
        rospy.wait_for_service('/robot_enable')
        try:
            enable_robot_service = rospy.ServiceProxy('/robot_enable', Trigger)
            response = enable_robot_service(
            )  # std_srvs/Trigger has empty request
            return response.success, response.message
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return False, str(e)

    def disable(self) -> None:
        rospy.wait_for_service('/robot_disable')
        try:
            enable_robot_service = rospy.ServiceProxy('/robot_disable',
                                                      Trigger)
            response = enable_robot_service(
            )  # std_srvs/Trigger has empty request
            return response.success, response.message
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return False, str(e)

    def set_planner(self, planner_name: str):
        '''
        Set the planner to be used by the MoveGroupCommander object
        planner_name[str]: Name of the planner to be used by the MoveGroupCommander object
                            options =   Planner.PTP, Planner.LIN etc
        '''
        self.planner = planner_name
        self.group.set_planner_id(planner_name)
        print("Planner set to: ", self.group.get_planner_id())

    def get_tool_pose(self) -> PoseStamped:
        """ Get the current pose of the tool

        Raises:
            SystemError: 

        Returns:
            PoseStamped: The current pose of the tool
        """

        posestamped = self.group.get_current_pose(self.tooltip_frame)

        # When moveit can't fetch robot state it silently returns a pose at 0 and logs to stderr
        # This checks that case and raises an error for the user
        p = posestamped.pose
        axis = [
            p.position.x,
            p.position.y,
            p.position.z,
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
        ]
        if all(axis) == 0 and p.orientation.w == 1:
            raise SystemError("Couldn't fetch robot state, please check logs")

        return posestamped

    def get_joint_positions(self) -> Dict[str, float]:
        """ Get the current joint positions of the robot

        Returns:
            Dict[str, float]: A dictionary of joint names and their corresponding positions
        """
        return dict(
            zip(self.group.get_active_joints(),
                self.group.get_current_joint_values()))

    def _reset_state(self) -> None:
        """
        Reset the state of the robot to the current state of the robot.
        """
        self.group.stop()
        self.group.set_start_state_to_current_state()
        self.group.set_goal_tolerance(0.0001)
        self.group.set_num_planning_attempts(Robot.PLANNING_ATTEMPTS)

        self.group.set_planner_id("PTP")

        self.group.clear_pose_targets()
        self._target_joint_values = None
        self._trajectory = None
        self.planning_time = 0.0

    def get_joint_limits(self) -> Dict[str, Joint]:
        return {
            j: Joint.from_robot(self.robot.get_joint(j))
            for j in self.group.get_active_joints()
        }



    def _plan(self) -> None:
        success, self._trajectory, self.planning_time, error_code = self.group.plan(
        )

        if not success:
            self._trajectory = None
            print("Planning failed with error code: {}".format(error_code))
            raise PlanningError(error_code.val)
            
        else:
            self._target_joint_values = self._trajectory.joint_trajectory.points[
                -1].positions
        return success

    def plan_linear(self,
                    pose: Pose,
                    velocity: float = 0.1,
                    acceleration: float = 0.1) -> None:
        """ 
        Plan a linear motion to a pose.
        
        Args:
            pose [Pose]: The target pose
            velocity [float]: The velocity scaling factor between 0 and 1
            acceleration [float]: The acceleration scaling factor of the robot between 0 and 1
        """
        self._reset_state()
        velocity, acceleration = check_vel_acc_limits(velocity, acceleration)

        self.set_planner(Planner.LIN)

        self.group.set_max_acceleration_scaling_factor(acceleration)
        self.group.set_max_velocity_scaling_factor(velocity)

        self.group.set_pose_target(pose)
        
        self._plan()
        
            

    def plan_ptp(self,
                 pose: Pose,
                 velocity: float = 0.1,
                 acceleration: float = 0.1) -> None:
        """
        Move the robot to a pose using point to point motion.
        
        Args:
            pose [Pose]: The target pose
            velocity [float]: The velocity scaling factor between 0 and 1
            acceleration [float]: The acceleration scaling factor of the robot between 0 and 1
        
        """
        self._reset_state()
        velocity, acceleration = check_vel_acc_limits(velocity, acceleration)

        self.set_planner(Planner.PTP)

        self.group.set_max_acceleration_scaling_factor(acceleration)
        self.group.set_max_velocity_scaling_factor(velocity)

        self.group.set_pose_target(pose)
        self._plan()

    def plan_cartesian_path(self,
                            pose_list: List[Pose],
                            velocity: float = 0.1,
                            acceleration: float = 0.1,
                            planner: str = None,
                            step_size: float = 0.001) -> None:
        """
        Move the robot to a pose using linear motion (for single pose) or spline motion (List of poses) in cartesian using specified planner other than the LIN and PTP planners.
        
        Args:
            pose List[Pose]: The target poses
            velocity [float]: The velocity scaling factor between 0 and 1
            acceleration [float]: The acceleration scaling factor of the robot between 0 and 1
            planner [str]: The planner to use
            step_size [float]: The step size of the cartesian path planner in meters. Defaults to 0.001
        """
        self._reset_state()
        velocity, acceleration = check_vel_acc_limits(velocity, acceleration)

        if planner:
            self.set_planner(planner)
        else:
            # self.set_planner(Robot.OMPL_DEFAULT)
            self.set_planner(Planner.SPARS)

        self.group.set_max_acceleration_scaling_factor(acceleration)
        self.group.set_max_velocity_scaling_factor(velocity)

        success = False  # Whether the motion planning succeeded
        waypoints = [_pose for _pose in pose_list]
        fraction = 0.0  # Initialization of the fraction of achieved distance
        attempts = 0  # Initialization of the number of attempts
        while fraction < 1.0 and attempts < 150:
            (plan, fraction) = self.group.compute_cartesian_path(
                waypoints, step_size, 0.0)
            attempts += 1
            if attempts % 20 == 0:
                rospy.loginfo("Still trying motion planning after " +
                              str(attempts) + " attempts...")
            if fraction == 1.0:
                rospy.loginfo("Motion planning succeeded after " +
                              str(attempts) + " attempts")
                success = True
            else:
                success = False
        if success:
            self._trajectory = plan
            self._target_joint_values = self._trajectory.joint_trajectory.points[
                -1].positions
        else:
            rospy.logwarn("Motion planning failed with only " + str(fraction))
            raise SplinePlanningError('Motion planning failed with only ' +
                                      str(fraction))

    def plan_joint_space_path(self,
                              pose: Pose,
                              velocity: float = 0.1,
                              acceleration: float = 0.1,
                              planner: str = None) -> None:
        """
        Move the robot to a pose using point to point motion using specified planner other than the LIN and PTP planners.
        
        Args:
            pose [Pose]: The target pose.
            velocity [float]: The velocity scaling factor between 0 and 1.
            acceleration [float]: The acceleration scaling factor of the robot between 0 and 1.
            planner [str]: The planner to use.
        """

        self._reset_state()
        velocity, acceleration = check_vel_acc_limits(velocity, acceleration)

        if planner:
            self.set_planner(planner)
        else:
            self.set_planner(Robot.OMPL_DEFAULT)

        # self.group.set_max_acceleration_scaling_factor(acceleration)
        # self.group.set_max_velocity_scaling_factor(velocity)

        self.group.set_pose_target(pose)
        self._plan()

    def move_linear(self,
                    pose: Pose,
                    velocity: float = 0.1,
                    acceleration: float = 0.1,
                    block: bool = True) -> None:
        """
        Move the robot to a pose using linear motion.
        
        Args:
            pose [Pose]: The target pose
            velocity [float]: The velocity scaling factor between 0 and 1
            acceleration [float]: The acceleration scaling factor of the robot between 0 and 1
            block [bool]: Whether to block until the motion is finished
        """
        self.plan_linear(pose, velocity, acceleration)
        success = self.execute_plan(block=block)
        return success

    def move_ptp(self,
                 pose: Pose,
                 velocity: float = 0.1,
                 acceleration: float = 0.1,
                 block: bool = True) -> None:
        """
        Move the robot to a pose using point to point motion.
        
        Args:
            pose [Pose]: The target pose.
            velocity [float]: The velocity scaling factor between 0 and 1.
            acceleration [float]: The acceleration scaling factor of the robot between 0 and 1.
            block [bool]: Whether to block until the motion is finished.
        
        """
        self.plan_ptp(pose, velocity, acceleration)
        success = self.execute_plan(block=block)
        return success

    def move_cartesian_path(self,
                            pose_list: List[Pose],
                            velocity: float = 0.1,
                            acceleration: float = 0.1,
                            planner: str = None,
                            step_size: float = 0.001,
                            block: bool = True) -> None:
        """
        Move the robot to a pose using linear motion (for single pose) or spline motion (List of poses) in cartesian using specified planner other than the LIN and PTP planners.
        
        Args:
            pose List[Pose]: The target poses
            velocity [float]: The velocity scaling factor between 0 and 1
            acceleration [float]: The acceleration scaling factor of the robot between 0 and 1
            planner [str]: The planner to use
            step_size [float]: The step size of the cartesian path planner in meters. Defaults to 0.001
            block [bool]: Whether to block until the motion is finished.
        """
        self.plan_cartesian_path(pose_list, velocity, acceleration, planner,
                                 step_size)
        success = self.execute_plan(block=block)
        return success

    def move_joint_space(self,
                         pose: Pose,
                         velocity: float = 0.1,
                         acceleration: float = 0.1,
                         planner: str = None,
                         block: bool = True) -> None:
        """
        Move the robot to a pose using point to point motion using specified planner other than the LIN and PTP planners.
        
        Args:
            pose [Pose]: The target pose.
            velocity [float]: The velocity scaling factor between 0 and 1.
            acceleration [float]: The acceleration scaling factor of the robot between 0 and 1.
            planner [str]: The planner to use.
            block [bool]: Whether to block until the motion is finished.
        """
        self.plan_joint_space_path(pose, velocity, acceleration, planner)
        success = self.execute_plan(block=block)
        return success

    def move_multi_joint_space(self,
                               pose_list: List[Pose],
                               velocity: float = 0.1,
                               acceleration: float = 0.1,
                               planner: str = None,
                               block: bool = True) -> None:
        """
        Plan multiple joint space paths and execute them sequentially.

        Args:
            pose [Pose]: The target pose.
            velocity [float]: The velocity scaling factor between 0 and 1.
            acceleration [float]: The acceleration scaling factor of the robot between 0 and 1.
            planner [str]: The planner to use.
            block [bool]: Whether to block until the motion is finished.
        """

        plans = []
        target_joint_values = []
        for pose in pose_list:
            self.plan_joint_space_path(pose, velocity, acceleration, planner)
            plans.append(self._trajectory)
            target_joint_values.append(self._target_joint_values)

        self._trajectory = None
        self._target_joint_values = None

        all_success = []
        for trajectory, target_joint_values in zip(plans, target_joint_values):
            self._trajectory = trajectory
            self._target_joint_values = target_joint_values
            success = self.execute_plan(block=block)
            all_success.append(success)
        return all_success.count(True) == len(all_success)

    def move_joint(self,
                   positions: List[float],
                   velocity: float = 0.1,
                   acceleration: float = 0.1,
                   block: bool = True) -> None:
        """
        Move the robot to a joint position.
        
        Args:
            positions [List[float]]: The target joint position.
            velocity [float]: The velocity scaling factor between 0 and 1.
            acceleration [float]: The acceleration scaling factor of the robot between 0 and 1.
            block [bool]: Whether to block until the motion is finished.
        """

        self._reset_state()

        active_joints = self.group.get_active_joints()
        if len(positions) != len(active_joints):
            raise ValueError(
                f"positions array contains {len(positions)} joints"
                f" where as this group has {len(active_joints)} joints")
        joint_limits = self.get_joint_limits()

        for i, val in enumerate(positions):
            joint_name = active_joints[i]
            joint = joint_limits[joint_name]
            clamp(val, joint.lower_bound, joint.upper_bound)

        velocity, acceleration = check_vel_acc_limits(velocity, acceleration)
        self.group.set_max_acceleration_scaling_factor(acceleration)
        self.group.set_max_velocity_scaling_factor(velocity)

        self.group.set_joint_value_target(positions)
        self._plan()
        success = self.execute_plan(block=block)
        return success

    @require_trajectory
    def execute_plan(self, block: bool = True) -> None:
        self.group.execute(self._trajectory, block)

        if not block:
            return

        if not self._reached_pose():
            rospy.logwarn("The robot did not reach it's target pose")
        return self._reached_pose()

    @require_trajectory
    def _reached_pose(self) -> bool:
        target_joint_values: List[float] = cast(List[float],
                                                self._target_joint_values)
        # print(target_joint_values)
        # print(self.get_joint_positions().values())

        distance = [
            abs(target - current)
            for target, current in zip(target_joint_values,
                                       self.get_joint_positions().values())
        ]

        if [i for i in distance if i >= self.JOINT_ERROR_THRESHOLD]:
            return False

        return sum(distance) / len(
            target_joint_values) <= self.JOINT_MEAN_ERROR_THRESHOLD

    def in_motion(self, ) -> bool:
        """
        Returns whether the robot is currently moving or not.
        
        """
        return not self._reached_pose()


if __name__ == '__main__':
    rospy.init_node('robot_control', anonymous=True)

    robot = Robot()
    tool_tip = robot.get_tool_pose()
    print(tool_tip)

