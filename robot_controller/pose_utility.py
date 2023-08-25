#!/usr/bin/env python3
from geometry_msgs.msg import Pose, Quaternion
import tf
import numpy as np
from tf.transformations import quaternion_from_euler
import tf.transformations as tft
from typing import Any, Callable, Dict, List, Optional, cast
from scipy.interpolate import CubicSpline, UnivariateSpline

class PoseConverter:

    def __init__(self,
                 position,
                 rpy: List[float] = None,
                 quaternion: List[float] = None):
        """Converts position, rpy or quaternion to a Pose object

        Args:
            position (_type_): cartesian position [x, y, z]
            rpy (List[float], optional): roll, pitch, yaw. Defaults to None.
            quaternion (List[float], optional): quaternion [x, y, z, w]. Defaults to None.
        """
        self.pose = Pose()
        self.pose.position.x = position[0]
        self.pose.position.y = position[1]
        self.pose.position.z = position[2]
        if rpy:
            qx, qy, qz, qw = quaternion_from_euler(rpy[2],
                                                   rpy[1],
                                                   rpy[0],
                                                   axes="szyx")
        elif quaternion:
            qx, qy, qz, qw = quaternion

        self.pose.orientation.x = qx
        self.pose.orientation.y = qy
        self.pose.orientation.z = qz
        self.pose.orientation.w = qw

    def get_pose(self):
        return self.pose


def calculate_relative_pose(current_pose: Pose,
                            position: List[float] = [0, 0, 0],
                            rpy: List[float] = [0, 0, 0]):
    """Calculates a new pose relative to the current pose

    Args:
        current_pose (Pose): The current pose
        position (List[float], optional): relative position [x, y, z]. Defaults to [0, 0, 0].
        rpy (List[float], optional): relative roll, pitch, yaw. Defaults to [0, 0, 0].
    """

    def get_pose_matrix(pose):
        translation = [pose.position.x, pose.position.y, pose.position.z]
        quaternion = [
            pose.orientation.x, pose.orientation.y, pose.orientation.z,
            pose.orientation.w
        ]
        return tf.transformations.translation_matrix(
            translation) @ tf.transformations.quaternion_matrix(quaternion)

    def matrix_to_pose(matrix):
        translation = tf.transformations.translation_from_matrix(matrix)
        quaternion = tf.transformations.quaternion_from_matrix(matrix)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = translation
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
        return pose

    # Convert position and RPY to a relative Pose object
    relative_pose = PoseConverter(position, rpy).get_pose()

    # Create a transformation matrix for the current pose
    current_pose_matrix = get_pose_matrix(current_pose)

    # Create a transformation matrix for the relative motion
    relative_motion_matrix = get_pose_matrix(relative_pose)

    # Apply the relative motion to the current pose
    new_pose_matrix = current_pose_matrix @ relative_motion_matrix

    # Convert the new transformation matrix back to a Pose object
    new_pose = matrix_to_pose(new_pose_matrix)

    return new_pose

