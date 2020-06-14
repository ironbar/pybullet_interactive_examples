import pybullet
import pybullet_data

pybullet.connect(pybullet.GUI)
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = pybullet.loadURDF("plane.urdf")
# robot = pybullet.loadURDF("../../kuka_experimental/kuka_kr210_support/urdf/kr210l150.urdf",
#                           [0, 0, 0], useFixedBase=1)  # use a fixed base!
robot = pybullet.loadURDF("../../kuka_experimental/kuka_lbr_iiwa_support/urdf/lbr_iiwa_14_r820.urdf",
                          [0, 0, 0], useFixedBase=1)  # use a fixed base!
pybullet.setGravity(0, 0, -9.81)
pybullet.setRealTimeSimulation(1) #this makes the simulation real time
# pybullet.setJointMotorControlArray(robot, range(6), pybullet.POSITION_CONTROL,
#                                    targetPositions=[0]*6)

position_idx = []
position_idx.append(pybullet.addUserDebugParameter('x', -2, 2, 0.1))
position_idx.append(pybullet.addUserDebugParameter('y', -2, 2, 0.1))
position_idx.append(pybullet.addUserDebugParameter('z', 0.2, 2, 1))

orientation_idx = []
orientation_idx.append(pybullet.addUserDebugParameter('euler 1', -3.14, 3.14, 0))
orientation_idx.append(pybullet.addUserDebugParameter('euler 2', -3.14, 3.14, 0))
orientation_idx.append(pybullet.addUserDebugParameter('euler 2', -3.14, 3.14, 0))



while 1:
    euler = [pybullet.readUserDebugParameter(idx) for idx in orientation_idx]
    orientation = pybullet.getQuaternionFromEuler(euler)
    position = [pybullet.readUserDebugParameter(idx) for idx in position_idx]
    target_positions = pybullet.calculateInverseKinematics(robot, 7, position, orientation)
    # target_positions = pybullet.calculateInverseKinematics(robot, 7, position)
    pybullet.setJointMotorControlArray(robot, range(7), pybullet.POSITION_CONTROL,
                                       targetPositions=target_positions)
