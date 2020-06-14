import pybullet
import pybullet_data

pybullet.connect(pybullet.GUI)
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = pybullet.loadURDF("plane.urdf")
robot = pybullet.loadURDF("../kuka_experimental/kuka_kr210_support/urdf/kr210l150.urdf",
                          [0, 0, 0], useFixedBase=1)  # use a fixed base!
pybullet.setGravity(0, 0, -9.81)
pybullet.setRealTimeSimulation(1) #this makes the simulation real time

pybullet.addUserDebugParameter('x')

while 1:
    x = pybullet.readUserDebugParameter(0)
    pybullet.setJointMotorControlArray(robot, range(6), pybullet.POSITION_CONTROL,
                                       targetPositions=[x] * 6)
