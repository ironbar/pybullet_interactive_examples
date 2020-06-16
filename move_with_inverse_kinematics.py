import sys
import argparse
import pybullet
import pybullet_data

def main():
    args = parse_args()
    pybullet.connect(pybullet.GUI)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane = pybullet.loadURDF("plane.urdf")
    robot = pybullet.loadURDF(args.robot_path, [0, 0, 0], useFixedBase=1)  # use a fixed base!
    pybullet.setGravity(0, 0, -9.81)
    pybullet.setRealTimeSimulation(1) #this makes the simulation real time

    position_idx = []
    position_idx.append(pybullet.addUserDebugParameter('x', -2, 2, 0.1))
    position_idx.append(pybullet.addUserDebugParameter('y', -2, 2, 0.1))
    position_idx.append(pybullet.addUserDebugParameter('z', 0.2, 2, 1))

    orientation_idx = []
    orientation_idx.append(pybullet.addUserDebugParameter('euler 1', -3.14, 3.14, 0))
    orientation_idx.append(pybullet.addUserDebugParameter('euler 2', -3.14, 3.14, 0))
    orientation_idx.append(pybullet.addUserDebugParameter('euler 2', -3.14, 3.14, 0))

    n_joints = pybullet.getNumJoints(robot)
    print('The number of joints in the robot is: %i' % n_joints)
    while 1:
        euler = [pybullet.readUserDebugParameter(idx) for idx in orientation_idx]
        orientation = pybullet.getQuaternionFromEuler(euler)
        position = [pybullet.readUserDebugParameter(idx) for idx in position_idx]
        target_positions = pybullet.calculateInverseKinematics(robot, n_joints-2, position, orientation)
        pybullet.setJointMotorControlArray(robot, range(len(target_positions)), pybullet.POSITION_CONTROL,
                                           targetPositions=target_positions)

def parse_args():
    parser = argparse.ArgumentParser(
        description='Allows to move each joint of the robot independently.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-r', '--robot_path', help='Path to the urdf model of the robot',
                        default='../kuka_experimental/kuka_lbr_iiwa_support/urdf/lbr_iiwa_14_r820.urdf')
    return parser.parse_args(sys.argv[1:])

if __name__ == '__main__':
    main()
