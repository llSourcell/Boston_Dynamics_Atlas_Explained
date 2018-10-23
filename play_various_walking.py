"""
bipedal robot walking simulation

by Einsbon (Sunbin Kim)
- GitHub: https://github.com/Einsbon
- Youtube:  https://www.youtube.com/channel/UCt7FZ-8uzV_jHJiKp3NlHvg
- Blog: https://blog.naver.com/einsbon
"""

import pybullet as p
import time
from time import sleep
import pybullet_data
import numpy as np
import math
import os

import motorController
import walkGenerator

# motor setting
motor_kp = 0.5
motor_kd = 0.5
motor_torque = 1.5
motor_max_velocity = 5.0

# physics parameter setting
fixedTimeStep = 1./1000
numSolverIterations = 200

physicsClient = p.connect(p.GUI)
p.setTimeStep(fixedTimeStep)
p.setPhysicsEngineParameter(numSolverIterations=numSolverIterations)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # to load plane.urdf

p.setGravity(0, 0, 0)
p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=10, cameraPitch=-5, cameraTargetPosition=[0.3, 0.5, 0.1])

# samurai.urdf  plane.urdf
planeId = p.loadSDF('stadium.sdf')

robot = p.loadURDF(os.path.abspath(os.path.dirname(__file__))+'/humanoid_leg_12dof.7.urdf', [0, 0, 0.05],
                   p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=False)

controller = motorController.MotorController(
    robot, physicsClient, fixedTimeStep, motor_kp, motor_kd, motor_torque, motor_max_velocity)

walk = walkGenerator.WalkGenerator()
walk.setWalkParameter(bodyMovePoint=8, legMovePoint=8, h=50, l=90, sit=40, swayBody=45, swayFoot=0,
                      bodyPositionXPlus=5, swayShift=3, weightStart=0.4, weightEnd=0.6, stepTime=0.06, damping=0.0, incline=0.0)
walk.generate()
walk.inverseKinematicsAll()

actionTime = walk._stepTime
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)
controller.setMotorsAngleInFixedTimestep(walk.walkPointStartRightInverse[0], 1, 0)


waitTime = 1
repeatTime = int(waitTime/fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()
    # time.sleep(fixedTimeStep)

p.setGravity(0, 0, -9.8)


# walk 8 steps
# start walking. right foot step
for i in range(np.size(walk.walkPointStartRightInverse, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkPointStartRightInverse[i], actionTime, 0)
for i in range(2):
    # left foot step
    for i in range(np.size(walk.walkPointLeftStepInverse, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkPointLeftStepInverse[i], actionTime, 0)
    # right foot step
    for i in range(np.size(walk.walkPointRightStepInverse, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkPointRightStepInverse[i], actionTime, 0)
# end walking. left
for i in range(np.size(walk.walkPointEndLeftInverse, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkPointEndLeftInverse[i], actionTime, 0)


# rest 2 seconds
waitTime = 2
repeatTime = int(waitTime/fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

########################################################
p.resetBasePositionAndOrientation(robot,  [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))
walk.setWalkParameter(bodyMovePoint=8, legMovePoint=8, h=50, l=90, sit=70, swayBody=45, swayFoot=0,
                      bodyPositionXPlus=5, swayShift=3, weightStart=0.4, weightEnd=0.6, stepTime=0.06, damping=0.0, incline=0.0)
walk.generate()
walk.inverseKinematicsAll()
actionTime = walk._stepTime
controller.setMotorsAngleInFixedTimestep(walk.walkPointStartRightInverse[0], 2, 0)

waitTime = 1
repeatTime = int(waitTime/fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

p.setGravity(0, 0, -9.8)


for i in range(np.size(walk.walkPointStartRightInverse, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkPointStartRightInverse[i], actionTime, 0)
for i in range(2):
    for i in range(np.size(walk.walkPointLeftStepInverse, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkPointLeftStepInverse[i], actionTime, 0)
    for i in range(np.size(walk.walkPointRightStepInverse, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkPointRightStepInverse[i], actionTime, 0)
for i in range(np.size(walk.walkPointEndLeftInverse, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkPointEndLeftInverse[i], actionTime, 0)

waitTime = 2
repeatTime = int(waitTime/fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()


########################################################
p.resetBasePositionAndOrientation(robot,  [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))

walk.setWalkParameter(bodyMovePoint=8, legMovePoint=8, h=20, l=40, sit=40, swayBody=30, swayFoot=0,
                      bodyPositionXPlus=5, swayShift=3, weightStart=0.4, weightEnd=0.6, stepTime=0.03, damping=0.0, incline=0.0)
walk.generate()
walk.inverseKinematicsAll()
actionTime = walk._stepTime
controller.setMotorsAngleInFixedTimestep(walk.walkPointStartRightInverse[0], 2, 0)

waitTime = 1
repeatTime = int(waitTime/fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

for i in range(np.size(walk.walkPointStartRightInverse, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkPointStartRightInverse[i], actionTime, 0)
for i in range(2):
    for i in range(np.size(walk.walkPointLeftStepInverse, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkPointLeftStepInverse[i], actionTime, 0)
    for i in range(np.size(walk.walkPointRightStepInverse, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkPointRightStepInverse[i], actionTime, 0)
for i in range(np.size(walk.walkPointEndLeftInverse, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkPointEndLeftInverse[i], actionTime, 0)

waitTime = 2
repeatTime = int(waitTime/fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()


########################################################
p.resetBasePositionAndOrientation(robot,  [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))

walk.setWalkParameter(bodyMovePoint=8, legMovePoint=8, h=50, l=140, sit=40, swayBody=50, swayFoot=0,
                      bodyPositionXPlus=-2, swayShift=3, weightStart=0.4, weightEnd=0.6, stepTime=0.12, damping=0.0, incline=0.0)
walk.generate()
walk.inverseKinematicsAll()
actionTime = walk._stepTime
controller.setMotorsAngleInFixedTimestep(walk.walkPointStartRightInverse[0], 2, 0)


waitTime = 1
repeatTime = int(waitTime/fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

for i in range(np.size(walk.walkPointStartRightInverse, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkPointStartRightInverse[i], actionTime, 0)
for i in range(2):
    for i in range(np.size(walk.walkPointLeftStepInverse, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkPointLeftStepInverse[i], actionTime, 0)
    for i in range(np.size(walk.walkPointRightStepInverse, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkPointRightStepInverse[i], actionTime, 0)
for i in range(np.size(walk.walkPointEndLeftInverse, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkPointEndLeftInverse[i], actionTime, 0)


waitTime = 2
repeatTime = int(waitTime/fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()


########################################################
p.resetBasePositionAndOrientation(robot,  [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))

walk.setWalkParameter(bodyMovePoint=8, legMovePoint=8, h=40, l=70, sit=40, swayBody=45, swayFoot=0,
                      bodyPositionXPlus=-40, swayShift=3, weightStart=0.4, weightEnd=0.6, stepTime=0.06, damping=0.0, incline=0.0)
walk.generate()
walk.inverseKinematicsAll()
actionTime = walk._stepTime
controller.setMotorsAngleInFixedTimestep(walk.walkPointStartRightInverse[0], 2, 0)

waitTime = 1
repeatTime = int(waitTime/fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

for i in range(np.size(walk.walkPointStartRightInverse, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkPointStartRightInverse[i], actionTime, 0)
for i in range(2):
    for i in range(np.size(walk.walkPointLeftStepInverse, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkPointLeftStepInverse[i], actionTime, 0)
    for i in range(np.size(walk.walkPointRightStepInverse, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkPointRightStepInverse[i], actionTime, 0)
for i in range(np.size(walk.walkPointEndLeftInverse, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkPointEndLeftInverse[i], actionTime, 0)

waitTime = 2
repeatTime = int(waitTime/fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

########################################################
p.resetBasePositionAndOrientation(robot,  [0, 0, 0.], p.getQuaternionFromEuler([0, 0, 0]))

walk.setWalkParameter(bodyMovePoint=8, legMovePoint=8, h=50, l=-90, sit=40, swayBody=45, swayFoot=0,
                      bodyPositionXPlus=0, swayShift=3, weightStart=0.4, weightEnd=0.6, stepTime=0.06, damping=0.0, incline=0.0)
walk.generate()
walk.inverseKinematicsAll()
actionTime = walk._stepTime
controller.setMotorsAngleInFixedTimestep(walk.walkPointStartRightInverse[0], 2, 0)

waitTime = 1
repeatTime = int(waitTime/fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

for i in range(np.size(walk.walkPointStartRightInverse, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkPointStartRightInverse[i], actionTime, 0)
for i in range(2):  # repeat twice
    # left foot step
    for i in range(np.size(walk.walkPointLeftStepInverse, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkPointLeftStepInverse[i], actionTime, 0)
    for i in range(np.size(walk.walkPointRightStepInverse, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkPointRightStepInverse[i], actionTime, 0)
for i in range(np.size(walk.walkPointEndLeftInverse, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkPointEndLeftInverse[i], actionTime, 0)

waitTime = 2
repeatTime = int(waitTime/fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

########################################################
p.resetBasePositionAndOrientation(robot,  [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))

walk.setWalkParameter(bodyMovePoint=8, legMovePoint=8, h=50, l=0, sit=40, swayBody=45, swayFoot=0,
                      bodyPositionXPlus=5, swayShift=3, weightStart=0.4, weightEnd=0.6, stepTime=0.06, damping=0.0, incline=0.0)
walk.generate()
walk.inverseKinematicsAll()
actionTime = walk._stepTime
controller.setMotorsAngleInFixedTimestep(walk.walkPointStartRightInverse[0], 2, 0)

waitTime = 1
repeatTime = int(waitTime/fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

# Turn function is not accurate yet.
for i in range(np.size(walk.walkPointStartRightInverse, 0)):
    controller.setMotorsAngleInFixedTimestep(
        walk.walkPointStartRightInverse[i]+walk.turnListUnfold[i]*0.3, actionTime, 0)
for i in range(3):
    for i in range(np.size(walk.walkPointLeftStepInverse, 0)):
        controller.setMotorsAngleInFixedTimestep(
            walk.walkPointLeftStepInverse[i]+walk.turnListFold[i]*0.3, actionTime, 0)
    for i in range(np.size(walk.walkPointRightStepInverse, 0)):
        controller.setMotorsAngleInFixedTimestep(
            walk.walkPointRightStepInverse[i]+walk.turnListUnfold[i]*0.3, actionTime, 0)
for i in range(np.size(walk.walkPointEndLeftInverse, 0)):
    controller.setMotorsAngleInFixedTimestep(
        walk.walkPointEndLeftInverse[i]+walk.turnListFold[i]*0.3, actionTime, 0)

waitTime = 2
repeatTime = int(waitTime/fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

########################################################
p.resetBasePositionAndOrientation(robot,  [0, 0, 0.0], p.getQuaternionFromEuler([0, 0, 0]))

walk.setWalkParameter(bodyMovePoint=8, legMovePoint=8, h=50, l=90, sit=40, swayBody=35, swayFoot=0,
                      bodyPositionXPlus=5, swayShift=3, weightStart=0.4, weightEnd=0.6, stepTime=0.06, damping=0.0, incline=0.0)
walk.generate()
walk.inverseKinematicsAll()
actionTime = walk._stepTime
controller.setMotorsAngleInFixedTimestep(walk.walkPointStartRightInverse[0], 2, 0)

waitTime = 1
repeatTime = int(waitTime/fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

for i in range(np.size(walk.walkPointStartRightInverse, 0)):
    controller.setMotorsAngleInFixedTimestep(
        walk.walkPointStartRightInverse[i]+walk.turnListUnfold[i]*0.3, actionTime, 0)
for i in range(3):
    for i in range(np.size(walk.walkPointLeftStepInverse, 0)):
        controller.setMotorsAngleInFixedTimestep(
            walk.walkPointLeftStepInverse[i]+walk.turnListFold[i]*0.3, actionTime, 0)
    for i in range(np.size(walk.walkPointRightStepInverse, 0)):
        controller.setMotorsAngleInFixedTimestep(
            walk.walkPointRightStepInverse[i]+walk.turnListUnfold[i]*0.3, actionTime, 0)
for i in range(np.size(walk.walkPointEndLeftInverse, 0)):
    controller.setMotorsAngleInFixedTimestep(
        walk.walkPointEndLeftInverse[i]+walk.turnListFold[i]*0.3, actionTime, 0)


########################################################
fixedTimeStep = 1/500
p.setTimeStep(fixedTimeStep)

giantRobot = p.loadURDF(os.path.abspath(os.path.dirname(__file__))+'/humanoid_leg_12dof.7.urdf',
                        [-4.9, -0.2, 0.01], p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=False, globalScaling=10)

waitTime = 2
repeatTime = int(waitTime/fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

for i in range(p.getNumJoints(giantRobot)):
    p.changeDynamics(giantRobot, i, lateralFriction=5)

controller2 = motorController.MotorController(
    giantRobot, physicsClient, fixedTimeStep, motor_kp, motor_kd, 50, motor_max_velocity)
walk.setWalkParameter(bodyMovePoint=8, legMovePoint=8, h=50, l=90, sit=30, swayBody=35, swayFoot=0,
                      bodyPositionXPlus=5, swayShift=3, weightStart=0.4, weightEnd=0.6, stepTime=0.08, damping=0.0, incline=0.0)
walk.generate()
walk.inverseKinematicsAll()
actionTime = walk._stepTime
controller2.setMotorsAngleInFixedTimestep(walk.walkPointStartRightInverse[0], 4, 0)

waitTime = 1
repeatTime = int(waitTime/fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

for i in range(np.size(walk.walkPointStartRightInverse, 0)):
    controller2.setMotorsAngleInFixedTimestep(walk.walkPointStartRightInverse[i], actionTime, 0)
for i in range(2):
    for i in range(np.size(walk.walkPointLeftStepInverse, 0)):
        controller2.setMotorsAngleInFixedTimestep(walk.walkPointLeftStepInverse[i], actionTime, 0)
    for i in range(np.size(walk.walkPointRightStepInverse, 0)):
        controller2.setMotorsAngleInFixedTimestep(walk.walkPointRightStepInverse[i], actionTime, 0)
for i in range(np.size(walk.walkPointEndLeftInverse, 0)):
    controller2.setMotorsAngleInFixedTimestep(walk.walkPointEndLeftInverse[i], actionTime, 0)

controller2.setMotorsAngleInFixedTimestep(walk.inverseKinematicsPoint([-10, -50, 30], [-10, -50, 30]), 1, 0.5)
controller2.setMotorsAngleInFixedTimestep(walk.inverseKinematicsPoint([-80, -50, 50], [-10, -50, 30]), 1.5, 0.5)
controller2.setMotorsAngleInFixedTimestep(walk.inverseKinematicsPoint([-10, -50, 40], [0, -50, 30]),  0.4, 0)
controller2.setMotorsAngleInFixedTimestep(walk.inverseKinematicsPoint([60, -40, 40], [0, -40, 30]), 0.2, 3)
controller2.setMotorsAngleInFixedTimestep(walk.inverseKinematicsPoint([0, -40, 45], [0, -40, 30]), 1, 0)
controller2.setMotorsAngleInFixedTimestep(walk.inverseKinematicsPoint([0, 0, 10], [0, 0, 10]), 0.5, 50)
