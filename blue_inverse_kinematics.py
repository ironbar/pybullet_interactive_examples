import sys
import time
import argparse
from tqdm import tqdm
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

    robot.go_to_rest_pose()
    for _ in tqdm(range(10), desc='startup'):
        time.sleep(0.01)

    right_control = PositionControl(*robot.get_right_arm_position(), prefix='right')
    rigth_clamp_control = ClampControl(prefix='right')

    left_control = PositionControl(*robot.get_left_arm_position(), prefix='left')
    left_clamp_control = ClampControl(prefix='left')

    for idx in range(pybullet.getNumJoints(robot.id)):
        print(idx, [round(x, 2) for x in robot._get_link_state(idx)[0]])

    # robot.debug_arm_idx()
    # color_idx = pybullet.addUserDebugParameter('color idx', 0, len(robot.moving_joints_idx), 0)

    

    while 1:
        position, orientation = right_control.get_position()
        robot.move_right_arm(position, orientation)
        debug_position(position, robot.get_right_arm_position()[0])

        position, orientation = left_control.get_position()
        robot.move_left_arm(position, orientation)
        debug_position(position, robot.get_left_arm_position()[0])

        if rigth_clamp_control.close_clamp():
            robot.close_right_clamp()
        else:
            robot.open_right_clamp()
        if left_clamp_control.close_clamp():
            robot.close_left_clamp()
        else:
            robot.open_left_clamp()

        # debug_motors(robot, target_positions, range(7))

        # for idx in robot.moving_joints_idx:
        #     pybullet.setDebugObjectColor(robot.id, idx, [0, 0, 0])
        # pybullet.setDebugObjectColor(robot.id, robot.moving_joints_idx[int(pybullet.readUserDebugParameter(color_idx))], [0, 1, 0])

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

def debug_position(goal, source):
    pybullet.addUserDebugLine(
        goal, source, lineColorRGB=[1, 0, 0], lifeTime=1, lineWidth=2)

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


class ClampControl():

    def __init__(self, prefix):
        self.id = pybullet.addUserDebugParameter('%s clamp' % prefix, 0, -1, 0)
        self.value = 0

    def close_clamp(self):
        return pybullet.readUserDebugParameter(self.id) % 2

def parse_args():
    parser = argparse.ArgumentParser(
        description='Allows to move each joint of the robot independently.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-r', '--robot_path', help='Path to the urdf model of the robot',
                        default='../blue_core/blue_descriptions/robots/blue_full_v1.urdf')
    return parser.parse_args(sys.argv[1:])

if __name__ == '__main__':
    main()
