# E206 Motion Planning

# AFP planner
# C Clark

import math
import matplotlib.pyplot as plt
from APF_utilities import *
from agent_0 import *
from agent_1 import *

# Constants
default_agent_radius = 0.5 #m
initial_score = 1000
goal_threshold = 0.5 #m

class Pose():
  x = 0
  y = 0
  theta = 0
  
  def __init__(self, x, y, theta):
    self.x = x
    self.y = y
    self.theta = theta
    
  def distance_to(self, pose):
    return math.sqrt((self.x - pose.x)**2 + (self.y - pose.y)**2)

class APFAgent():
  pose = Pose(0, 0, 0)
  goal_pose = Pose(0,0,0)
  v = 0
  w = 0
  id = -1
  K_w = 1
  K_v = 1
  w_max = 1
  v_max = 2
  radius = 0.5
  alive = True
  af0 = AgentForce_0()
  af1 = AgentForce_1()

  def __init__(self, pose, goal_pose, radius, id):
    """ Constructor
    """
    self.pose = pose
    self.goal_pose = goal_pose
    self.radius = radius
    self.id = id
    self.v = 0
    self.w = 0
    self.score = 0
    self.collided = False
    
  def update(self, delta_t, agent_list, object_list, world_radius):
    """ Function to update an agents state.
        Arguments:
          delta_t (float): The time step size in s.
          agent_list (list of APF_agents): The autonomous agents in the world.
          object_list (list of APF_agents): The objects in the world that agents should avoid.
          world_radius (float): The radius the world's workspace
    """
    if self.alive:
      apf_force_vector = self.generate_force_vector(agent_list, object_list, world_radius)
      self.actuate_robot(apf_force_vector, delta_t)
    
  def generate_force_vector(self, agent_list, object_list, world_radius):
    """ Function to plot the simulated world.
        Arguments:
          agent_list (list of APF_agents): The autonomous agents in the world.
          object_list (list of APF_agents): The objects in the world that agents should avoid.
          world_radius (float): The radius the world's workspace
        Returns:
          force_vector (list of 2 floats): The force vector the agent should follow for navigation.
    """
    if self.id == 0:
      return self.af0.generate_force_vector(self.id, agent_list, object_list, world_radius)
    if self.id == 1:
      return self.af1.generate_force_vector(self.id, agent_list, object_list, world_radius)
    if self.id == 2:
      return self.af0.generate_force_vector(self.id, agent_list, object_list, world_radius)
    if self.id == 3:
      return self.af0.generate_force_vector(self.id, agent_list, object_list, world_radius)
    if self.id == 4:
      return self.af0.generate_force_vector(self.id, agent_list, object_list, world_radius)
    if self.id == 5:
      return self.af0.generate_force_vector(self.id, agent_list, object_list, world_radius)
    if self.id == 6:
      return self.af0.generate_force_vector(self.id, agent_list, object_list, world_radius)
    if self.id == 7:
      return self.af0.generate_force_vector(self.id, agent_list, object_list, world_radius)
    if self.id == 8:
      return self.af0.generate_force_vector(self.id, agent_list, object_list, world_radius)
    if self.id == 9:
      return self.af0.generate_force_vector(self.id, agent_list, object_list, world_radius)

  def actuate_robot(self, apf_force_vector, delta_t):
    """ Convert a force vector to a control signal and actuate the agent like
        a robot.
        Arguments:
          apf_force_vector (list of 2 floats): The 2D force vector.
          delta_t (float): The time step size in s.
    """
    force_angle = math.atan2(apf_force_vector[1], apf_force_vector[0])
    v = self.K_v * math.sqrt(apf_force_vector[0]**2 + apf_force_vector[1]**2 )
    w = self.K_w * angle_diff(force_angle-self.pose.theta)
    v = max(min(v, self.v_max), -self.v_max)
    w = max(min(w, self.w_max), -self.w_max)
    self.pose.x = self.pose.x + v * math.cos(self.pose.theta)*delta_t
    self.pose.y = self.pose.y + v * math.sin(self.pose.theta)*delta_t
    self.pose.theta = self.pose.theta + w*delta_t
  
  def collision(self, agent):
    """ Function to check if the agent is in collision with another agent.
        Arguments:
          agent (APF_agent): The other agent there could be a collision with.
        Returns:
          collision (Bool): True if there is a collision.
    """
    dist = math.sqrt((self.pose.x - agent.pose.x)**2 + (self.pose.y - agent.pose.y)**2)
    return dist < self.radius + agent.radius and self.alive and agent.alive
    
  def remove(self):
    """ Function to reset the pose to be at the origin and set alive to false.
    """
    self.pose = Pose(0, 0, 0)
    self.alive = False
    
  def out_of_bounds(self, world_radius):
    """ Function to check if the agent is out of bounds.
        Arguments:
          world_radius (float): The radius of the world in m.
        Returns:
          out_of_bounds (Bool): True if the agent is out of bounds.
    """
    dist = math.sqrt(self.pose.x**2 + self.pose.y**2)
    return dist > world_radius - self.radius
    
  def at_goal(self):
    """ Function to check if the agent has reached its goal.
        Returns:
          at_goal (Bool): True if the agent is at its goal pose.
    """
    return self.pose.distance_to(self.goal_pose) < goal_threshold

  def update_score(self, time_step):
    """ Function to update an agents score based on goal reached and collisions.
        Arguments:
          time_step (float): The world's current time step in s.
    """
    if self.alive:
      if self.at_goal():
        self.score = initial_score - time_step
        self.remove()
        print("Agent ",self.id,"done with score",self.score)
      elif self.collided:
        self.score = time_step
        self.remove()
        print("Agent ",self.id,"done with score",self.score)
