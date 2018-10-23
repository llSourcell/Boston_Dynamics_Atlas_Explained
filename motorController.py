"""
This helps control motors in pybullet physics simulator.
https://github.com/Einsbon
https://blog.naver.com/einsbon
"""


import numpy as np
import time
from time import sleep
import pybullet as p


class MotorController:
    def __init__(self, robot, physicsClientId, timeStep, kp, kd, torque, max_velocity):
        self._robot = robot
        self._physicsClientId = physicsClientId
        jointNameToId = {}
        joint_id_list = []
        joint_pos_list = []
        self._joint_number = 0
        for i in range(p.getNumJoints(self._robot, physicsClientId=self._physicsClientId)):
            jointInfo = p.getJointInfo(self._robot, i, physicsClientId=self._physicsClientId)
            if jointInfo[2] == 0:
                joint_id_list.append(jointInfo[0])
                joint_pos_list.append(p.getJointState(
                    self._robot, jointInfo[0], physicsClientId=self._physicsClientId)[0])
                jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
                self._joint_number += 1
        self._joint_id = np.array(joint_id_list, dtype=np.int32)
        self._joint_targetPos = np.array(joint_pos_list, dtype=np.float)
        self._joint_currentPos = np.array(joint_pos_list, dtype=np.float)

        self._jointNameToId = jointNameToId
        self._kp = kp
        self._kd = kd
        self._torque = torque
        self._max_velocity = max_velocity
        self._timeStep = timeStep
        # print(self._joint_id)
        # print(self._joint_targetPos)
        # print(self._jointNameToId)

    def setRobot(self, robot, physicsClientId=None):
        self._robot = robot
        if physicsClientId != None:
            self._physicsClientId = physicsClientId

        joint_id_list = []
        joint_pos_list = []
        self._joint_number = 0
        for i in range(p.getNumJoints(robot)):
            jointInfo = p.getJointInfo(robot, i)
            if jointInfo[2] == 0:
                joint_id_list.append(jointInfo[0])
                joint_pos_list.append(p.getJointState(robot, jointInfo[0])[0])
                self._joint_number += 1
        print(self._joint_number)
        self._joint_id = np.array(joint_id_list, dtype=np.int32)
        self._joint_targetPos = np.array(joint_pos_list, dtype=np.float)
        self._joint_currentPos = np.array(joint_pos_list, dtype=np.float)

    def setMotorParameters(self, kp=None, kd=None, torque=None, maxVelocity=None, timeStep=None):
        if kp != None:
            self._kp = kp
        if kd != None:
            self._kd = kd
        if torque != None:
            self._torque = torque
        if maxVelocity != None:
            self._max_velocity = maxVelocity
        if timeStep != None:
            self._timeStep = timeStep

    def getRevoluteJoint_nameToId(self):
        return self._jointNameToId

    def getMotorAngle(self):
        for i in range(self._joint_number):
            self._joint_currentPos[i] = p.getJointState(
                self._robot, self._joint_id[i], physicsClientId=self._physicsClientId)[0]
        return self._joint_currentPos

    def setMotorAngle(self, motorTargetAngles):
        for i in range(self._joint_number):
            self._joint_targetPos[i] = motorTargetAngles[i]
            p.setJointMotorControl2(bodyIndex=self._robot, jointIndex=self._joint_id[i], controlMode=p.POSITION_CONTROL,
                                    targetPosition=self._joint_targetPos[i],
                                    positionGain=self._kp, velocityGain=self._kd, force=self._torque, maxVelocity=self._max_velocity, physicsClientId=self._physicsClientId)

    def setMotorsAngleInRealTimestep(self, motorTargetAngles, motorTargetTime, delayTime):
        if(motorTargetTime == 0):
            self._joint_targetPos = np.array(motorTargetAngles)
            for i in range(self._joint_number):
                p.setJointMotorControl2(bodyIndex=self._robot, jointIndex=self._joint_id[i], controlMode=p.POSITION_CONTROL,
                                        targetPosition=self._joint_targetPos[i],
                                        positionGain=self._kp, velocityGain=self._kd, force=self._torque, maxVelocity=self._max_velocity, physicsClientId=self._physicsClientId)
            time.sleep(delayTime)
        else:
            self._joint_currentPos = self._joint_targetPos
            self._joint_targetPos = np.array(motorTargetAngles)
            for i in range(self._joint_number):
                dydt = (self._joint_targetPos-self._joint_currentPos)/motorTargetTime
            internalTime = 0.0
            reft = time.time()
            while internalTime < motorTargetTime:
                internalTime = time.time() - reft
                for i in range(self._joint_number):
                    p.setJointMotorControl2(bodyIndex=self._robot, jointIndex=self._joint_id[i], controlMode=p.POSITION_CONTROL,
                                            targetPosition=self._joint_currentPos[i] + dydt[i] * internalTime,
                                            positionGain=self._kp, velocityGain=self._kd, force=self._torque, maxVelocity=self._max_velocity, physicsClientId=self._physicsClientId)

    def setMotorsAngleInFixedTimestep(self, motorTargetAngles, motorTargetTime, delayTime):
        if(motorTargetTime == 0):
            self._joint_targetPos = np.array(motorTargetAngles)
            for i in range(self._joint_number):
                p.setJointMotorControl2(bodyIndex=self._robot, jointIndex=self._joint_id[i], controlMode=p.POSITION_CONTROL,
                                        targetPosition=self._joint_targetPos[i],
                                        positionGain=self._kp, velocityGain=self._kd, force=self._torque, maxVelocity=self._max_velocity, physicsClientId=self._physicsClientId)
                p.stepSimulation(physicsClientId=self._physicsClientId)
                time.sleep(self._timeStep, physicsClientId=self._physicsClientId)
        else:
            self._joint_currentPos = self._joint_targetPos
            self._joint_targetPos = np.array(motorTargetAngles)
            for i in range(self._joint_number):
                dydt = (self._joint_targetPos-self._joint_currentPos)/motorTargetTime
            internalTime = 0.0
            while internalTime < motorTargetTime:
                internalTime += self._timeStep
                for i in range(self._joint_number):
                    p.setJointMotorControl2(bodyIndex=self._robot, jointIndex=self._joint_id[i], controlMode=p.POSITION_CONTROL,
                                            targetPosition=self._joint_currentPos[i] + dydt[i] * internalTime,
                                            positionGain=self._kp, velocityGain=self._kd, force=self._torque, maxVelocity=self._max_velocity, physicsClientId=self._physicsClientId)
                p.stepSimulation(physicsClientId=self._physicsClientId)

            if delayTime != 0:
                for _ in range(int(delayTime/self._timeStep)):
                    p.stepSimulation(physicsClientId=self._physicsClientId)
