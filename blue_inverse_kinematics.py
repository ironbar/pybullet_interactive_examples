import sys
import time
import argparse
import pybullet
import pybullet_data

from blue import BlueRobot

def main():
    args = parse_args()
    pybullet.connect(pybullet.GUI)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane = pybullet.loadURDF("plane.urdf")
    pybullet.setGravity(0, 0, -9.81)
    pybullet.setRealTimeSimulation(1) #this makes the simulation real time

    robot = BlueRobot(args.robot_path)

    right_control = PositionControl(*robot.get_right_arm_position(), prefix='right')
    left_control = PositionControl(*robot.get_left_arm_position(), prefix='left')

    for idx in range(pybullet.getNumJoints(robot.id)):
        print(idx, [round(x, 2) for x in robot._get_link_state(idx)[0]])

    while 1:
        robot.move_right_arm(*right_control.get_position())
        robot.move_left_arm(*left_control.get_position())

        # debug_motors(robot, target_positions, range(7))

def debug_motors(robot, target_positions, joint_idx):
    sep = '  \t'
    target_positions = [target_positions[idx] for idx in joint_idx]
    print('target_positions  ', sep.join([str(round(x, 2)) for x in target_positions]))
    current_positions = [pybullet.getJointState(robot.id, robot.moving_joints_idx[idx])[0] for idx in joint_idx]
    print('current_positions ', sep.join([str(round(x, 2)) for x in current_positions]))
    print('diff              ', sep.join([str(round(x-y, 2)) for x, y in zip(current_positions, target_positions)]))
    lower_limits = [robot.lower_limits[idx] for idx in joint_idx]
    print('lower_limits      ', sep.join([str(round(x, 2)) for x in lower_limits]))
    upper_limits = [robot.upper_limits[idx] for idx in joint_idx]
    print('upper_limits      ', sep.join([str(round(x, 2)) for x in upper_limits]))
    print()

class PositionControl():

    def __init__(self, initial_position, initial_orientation, prefix):
        position_idx = []
        x, y, z = initial_position[0], initial_position[1], initial_position[2]
        offset = 1
        position_idx.append(pybullet.addUserDebugParameter('%s x' % prefix, x - offset, x + offset, x))
        position_idx.append(pybullet.addUserDebugParameter('%s y' % prefix, y - offset, y + offset, y))
        position_idx.append(pybullet.addUserDebugParameter('%s z' % prefix, 0.2, 2, z))

        orientation_idx = []
        orientation_idx.append(pybullet.addUserDebugParameter('%s euler 1' % prefix, -3.14, 3.14, initial_orientation[0]))
        orientation_idx.append(pybullet.addUserDebugParameter('%s euler 2' % prefix, -3.14, 3.14, initial_orientation[1]))
        orientation_idx.append(pybullet.addUserDebugParameter('%s euler 3' % prefix, -3.14, 3.14, initial_orientation[2]))

        self.position_idx = position_idx
        self.orientation_idx = orientation_idx

    def get_position(self):
        orientation = [pybullet.readUserDebugParameter(idx) for idx in self.orientation_idx]
        position = [pybullet.readUserDebugParameter(idx) for idx in self.position_idx]
        return position, orientation


def parse_args():
    parser = argparse.ArgumentParser(
        description='Allows to move each joint of the robot independently.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-r', '--robot_path', help='Path to the urdf model of the robot',
                        default='../blue_core/blue_descriptions/robots/blue_full_v1.urdf')
    return parser.parse_args(sys.argv[1:])

if __name__ == '__main__':
    main()
