import gym
from gym import error, spaces, utils
from gym.utils import seeding

import os
import math
import numpy as np
import pybullet as p
import pybullet_data

VEL_MAX = 100.0

class FetchEnv(gym.Env):
  metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second' : 50
    }

  def __init__(self):
    self._observation = []
    self.action_space = spaces.Box(np.array([-VEL_MAX, -VEL_MAX, -3.14, -3.14, -3.14, -3.14, -3.14, -3.14, -3.14, -3.14, -3.14, -3.14]), np.array([VEL_MAX, VEL_MAX, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14]), dtype=np.float32)
    self.observation_space = spaces.Box(np.array([-5, -5, -math.pi]), np.array([5, 5, math.pi]))

    self.physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
    p.URDF_USE_MATERIAL_COLORS_FROM_MTL
    p.URDF_USE_MATERIAL_TRANSPARANCY_FROM_MTL
    self._seed()
    self.objects = []
    
  def set_parameters(self, time_step_size, objects):
    self.time_step_size = time_step_size
    self.objects = objects
    
  def _seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def _step(self, action):
    self._assign_mobile_vel(action)
    self._assign_manipulator_pos(action)
    p.stepSimulation()
    self._observation = self._compute_observation()
    reward = self._compute_reward()
    done = self._compute_done()

    self._envStepCounter += 1

    return np.array(self._observation), reward, done, {}

  def _reset(self):
    self.vt = 0
    self.vd = 0
    self._envStepCounter = 0

    p.resetSimulation()
    p.setGravity(0,0,-10) # m/s^2
    p.setTimeStep(self.time_step_size) # sec
    planeId = p.loadURDF("plane.urdf")
    for obj in self.objects:
      objectId = p.loadURDF("sphere2.urdf",[obj[0],obj[1],obj[2]],p.getQuaternionFromEuler([0,0,0]), globalScaling=obj[2])
    cubeStartPos = [0,0,0.001]
    cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])

    path = os.path.abspath(os.path.dirname(__file__))
    self.botId = p.loadURDF("./gym_fetch/envs/fetch.urdf",cubeStartPos, cubeStartOrientation)
    self._observation = self._compute_observation()
    return np.array(self._observation)

  def _assign_mobile_vel(self, action):
    p.setJointMotorControl2(bodyUniqueId=self.botId,
                            jointIndex=0,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=float(np.clip(action[0], -VEL_MAX, VEL_MAX)))
    p.setJointMotorControl2(bodyUniqueId=self.botId,
                            jointIndex=1,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=float(np.clip(action[1], -VEL_MAX, VEL_MAX)))

  def _assign_manipulator_pos(self, action):
    maxMotorForce = 2000
    p.setJointMotorControl2(bodyUniqueId=self.botId,
                            jointIndex=10, # shoulder_pan_joint
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=float(np.clip(action[2], -3.14, 3.14)),
                            force=maxMotorForce)
    p.setJointMotorControl2(bodyUniqueId=self.botId,
                            jointIndex=11, # shoulder_lift_joint
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=float(np.clip(action[3], -3.14, 3.14)),
                            force=maxMotorForce)
    p.setJointMotorControl2(bodyUniqueId=self.botId,
                            jointIndex=12, # upperarm_roll_joint
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=float(np.clip(action[4], -3.14, 3.14)),
                            force=maxMotorForce)
    p.setJointMotorControl2(bodyUniqueId=self.botId,
                            jointIndex=13, # elbow_flex_joint
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=float(np.clip(action[5], -3.14, 3.14)),
                            force=maxMotorForce)
    p.setJointMotorControl2(bodyUniqueId=self.botId,
                            jointIndex=14, # forearm_roll_joint
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=float(np.clip(action[6], -3.14, 3.14)),
                            force=maxMotorForce)
    p.setJointMotorControl2(bodyUniqueId=self.botId,
                            jointIndex=15, # wrist_flex_joint
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=float(np.clip(action[7], -3.14, 3.14)),
                            force=maxMotorForce)
    p.setJointMotorControl2(bodyUniqueId=self.botId,
                            jointIndex=16, # wrist_roll_joint
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=float(np.clip(action[8], -3.14, 3.14)),
                            force=maxMotorForce)
    p.setJointMotorControl2(bodyUniqueId=self.botId,
                            jointIndex=17, # gripper_axis_joint
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=0,
                            force=maxMotorForce)
    p.setJointMotorControl2(bodyUniqueId=self.botId,
                            jointIndex=18, # r_gripper_finger_joint
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=0,
                            force=maxMotorForce)
    p.setJointMotorControl2(bodyUniqueId=self.botId,
                            jointIndex=19, # l_gripper_finger_joint
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=0,
                            force=maxMotorForce)

  def _compute_observation(self):
    cubePos, cubeOrn = p.getBasePositionAndOrientation(self.botId)
    cubeEuler = p.getEulerFromQuaternion(cubeOrn)
    linear, angular = p.getBaseVelocity(self.botId)
    return [cubePos[0], cubePos[1], cubeEuler[2]]

  def _compute_reward(self):
    _, cubeOrn = p.getBasePositionAndOrientation(self.botId)
    cubeEuler = p.getEulerFromQuaternion(cubeOrn)
    # could also be pi/2 - abs(cubeEuler[0])
    return (1 - abs(cubeEuler[0])) * 0.1 -  abs(self.vt - self.vd) * 0.01

  def _compute_done(self):
    cubePos, _ = p.getBasePositionAndOrientation(self.botId)
    return cubePos[2] < 0.15 or self._envStepCounter >= 1500

  def _render(self, mode='human', close=False):
    pass
