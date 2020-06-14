import pybullet
import pybullet_data

pybullet.connect(pybullet.GUI)
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = pybullet.loadURDF("plane.urdf")
robot = pybullet.loadURDF("../kuka_experimental/kuka_kr210_support/urdf/kr210l150.urdf",
                          [0, 0, 0], useFixedBase=1)  # use a fixed base!
pybullet.setGravity(0, 0, -9.81)
pybullet.setRealTimeSimulation(1) #this makes the simulation real time

param_idx = []
for idx in range(6):
    joint_info = pybullet.getJointInfo(robot, idx)
    lower_limit, upper_limit = joint_info[8], joint_info[9]
    print('Joint %i has limits: (%.1f , %.1f)' % (idx, lower_limit, upper_limit))
    param_idx.append(pybullet.addUserDebugParameter('joint %i' % idx, lower_limit, upper_limit, 0))

while 1:
    target_positions = [pybullet.readUserDebugParameter(idx) for idx in param_idx]
    pybullet.setJointMotorControlArray(robot, range(6), pybullet.POSITION_CONTROL,
                                       targetPositions=target_positions)
