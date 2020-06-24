import sys
import time
import argparse
import pybullet

class BlueRobot():
    RIGHT_ARM_LINK_IDX = 7
    LEFT_ARM_LINK_IDX = 24

    def __init__(self, robot_path):
        flags = pybullet.URDF_USE_SELF_COLLISION | pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        self.id = pybullet.loadURDF(robot_path, [0, 0, 1], useFixedBase=1, flags=flags)
        self.kinematics_kwargs, self.moving_joints_idx = getJointRanges(self.id, False)
        self.lower_limits = self.kinematics_kwargs['lowerLimits']
        self.upper_limits = self.kinematics_kwargs['upperLimits']
        self.rest_poses = self.kinematics_kwargs['restPoses']
        self.n_moving_joints = len(self.moving_joints_idx)

    def get_right_arm_position(self):
        return self._get_link_state(self.RIGHT_ARM_LINK_IDX)

    def get_left_arm_position(self):
        return self._get_link_state(self.LEFT_ARM_LINK_IDX)

    def _get_link_state(self, link_idx):
        return get_link_state(self.id, link_idx)

    def move_right_arm(self, position, orientation):
        target_positions = self._inverse_kinematics(
            self.RIGHT_ARM_LINK_IDX, position, orientation)
        pybullet.setJointMotorControlArray(
            self.id, self.moving_joints_idx[:7], pybullet.POSITION_CONTROL,
            targetPositions=target_positions[:7])

    def move_left_arm(self, position, orientation):
        target_positions = self._inverse_kinematics(
            self.LEFT_ARM_LINK_IDX, position, orientation)
        pybullet.setJointMotorControlArray(
            self.id, self.moving_joints_idx[12:19], pybullet.POSITION_CONTROL,
            targetPositions=target_positions[12:19])

    def close_right_clamp(self):
        pybullet.setJointMotorControlArray(
            self.id, self.moving_joints_idx[8:12], pybullet.POSITION_CONTROL,
            targetPositions=[1, 0, 1, 0])

    def open_right_clamp(self):
        pybullet.setJointMotorControlArray(
            self.id, self.moving_joints_idx[8:12], pybullet.POSITION_CONTROL,
            targetPositions=[0]*4)

    def close_left_clamp(self):
        pybullet.setJointMotorControlArray(
            self.id, self.moving_joints_idx[20:24], pybullet.POSITION_CONTROL,
            targetPositions=[1, 0, 1, 0])

    def open_left_clamp(self):
        pybullet.setJointMotorControlArray(
            self.id, self.moving_joints_idx[20:24], pybullet.POSITION_CONTROL,
            targetPositions=[0]*4)

    def _inverse_kinematics(self, link_idx, position, orientation):
        if len(orientation) == 3:
            orientation = pybullet.getQuaternionFromEuler(orientation)
        target_positions = pybullet.calculateInverseKinematics(
            self.id,
            endEffectorLinkIndex=link_idx,
            targetPosition=position,
            targetOrientation=orientation,
            **self.kinematics_kwargs)
        return target_positions


def getJointRanges(bodyId, includeFixed=False):
    """
    https://github.com/erwincoumans/pybullet_robots/blob/master/baxter_ik_demo.py

    Parameters
    ----------
    bodyId : int
    includeFixed : bool
    Returns
    -------
    lowerLimits : [ float ] * numDofs
    upperLimits : [ float ] * numDofs
    jointRanges : [ float ] * numDofs
    restPoses : [ float ] * numDofs
    """

    lowerLimits, upperLimits, jointRanges, restPoses = [], [], [], []
    moving_joints_idx = []

    numJoints = pybullet.getNumJoints(bodyId)

    for i in range(numJoints):
        jointInfo = pybullet.getJointInfo(bodyId, i)

        if includeFixed or jointInfo[3] > -1:

            ll, ul = jointInfo[8:10]
            jr = ul - ll

            # For simplicity, assume resting state == initial state
            rp = pybullet.getJointState(bodyId, i)[0]
            # Instead of that I will use the rest state as the middle point for each motor
            # rp = ll + jr/4

            # lowerLimits.append(-2)
            # upperLimits.append(2)
            # jointRanges.append(2)
            lowerLimits.append(ll)
            upperLimits.append(ul)
            jointRanges.append(jr)
            restPoses.append(rp)
            moving_joints_idx.append(i)

    output = dict(
        lowerLimits=lowerLimits, upperLimits=upperLimits,
        jointRanges=jointRanges, restPoses=restPoses)
    return output, moving_joints_idx


def get_link_state(robot, link_idx):
    ret = pybullet.getLinkState(robot, link_idx)
    position, orientation = ret[0], pybullet.getEulerFromQuaternion(ret[1])
    return position, orientation