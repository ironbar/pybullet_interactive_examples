import sys
import argparse
import pybullet
import pybullet_data

def main():
    args = parse_args()
    pybullet.connect(pybullet.GUI)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane = pybullet.loadURDF("plane.urdf")
    flags = pybullet.URDF_USE_SELF_COLLISION | pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
    robot = pybullet.loadURDF(args.robot_path, [0, 0, 0], useFixedBase=1, flags=flags)  # use a fixed base!
    pybullet.setGravity(0, 0, -9.81)
    pybullet.setRealTimeSimulation(1) #this makes the simulation real time

    param_idx = []
    n_joints = pybullet.getNumJoints(robot)
    for idx in range(n_joints):
        joint_info = pybullet.getJointInfo(robot, idx)
        lower_limit, upper_limit = joint_info[8], joint_info[9]
        print('Joint %i has limits: (%.1f , %.1f)' % (idx, lower_limit, upper_limit))
        param_idx.append(pybullet.addUserDebugParameter('joint %i' % idx, lower_limit, upper_limit, 0))

    while 1:
        target_positions = [pybullet.readUserDebugParameter(idx) for idx in param_idx]
        pybullet.setJointMotorControlArray(robot, range(n_joints), pybullet.POSITION_CONTROL,
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
