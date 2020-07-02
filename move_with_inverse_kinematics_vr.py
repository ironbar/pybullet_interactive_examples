import sys
import argparse
import pybullet
import pybullet_data

from blue import get_link_state
from blue_inverse_kinematics import PositionControl, debug_position

def main():
    args = parse_args()
    pybullet.connect(pybullet.SHARED_MEMORY)
    pybullet.resetSimulation()
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    pybullet.loadURDF("plane.urdf")
    flags = pybullet.URDF_USE_SELF_COLLISION | pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
    robot = pybullet.loadURDF(args.robot_path, [0, 0, 0], useFixedBase=1, flags=flags)
    pybullet.setGravity(0, 0, -9.81)
    pybullet.setRealTimeSimulation(1) #this makes the simulation real time

    n_joints = pybullet.getNumJoints(robot)
    reference_link = n_joints-2
    position, orientation = get_link_state(robot, reference_link)

    print('The number of joints in the robot is: %i' % n_joints)
    while 1:

        events = pybullet.getVREvents()
        for event in events:
            _, position, orientation = event[:3]

            target_positions = pybullet.calculateInverseKinematics(robot, reference_link,
                                                                   position, orientation)
            pybullet.setJointMotorControlArray(
                robot, range(len(target_positions)), pybullet.POSITION_CONTROL,
                targetPositions=target_positions)
            current_position, _ = get_link_state(robot, reference_link)
            debug_position(position, current_position, 0.2)

def parse_args():
    description = """
    Allows to control the robot using VR. Only one controller is needed.

    Previously to launching this script the App_PhysicsServer_SharedMemory_VR needs to be launched
    with steam. Is is inside pybullet repo and it needs to be built. In my case it is the following
    command.

    /home/guillermo/.steam/ubuntu12_32/steam-runtime/run.sh /media/guillermo/Data/MEGA/mimo/bullet3/build_cmake/examples/SharedMemory/App_PhysicsServer_SharedMemory_VR
    """
    parser = argparse.ArgumentParser(
        description=description,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-r', '--robot_path', help='Path to the urdf model of the robot',
                        default='../kuka_experimental/kuka_lbr_iiwa_support/urdf/lbr_iiwa_14_r820.urdf')
    return parser.parse_args(sys.argv[1:])

if __name__ == '__main__':
    main()
