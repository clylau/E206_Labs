import numpy as np
import matplotlib.pyplot as plt
from utilities import *
import math
# from agent_0 import *
# from agent_1 import *

# Constants
default_agent_radius = 0.5 #m
initial_score = 1000
goal_threshold = 1 #m
wall_epsilon = 2


class AgentForce():
  
  def __init__(self, id, k_att = 10, k_rep = 1000000, rho_0 = 4): # Change k_rep to 10 for object collision
    self.id = id
    # self.k_att = 1
    # self.k_rep = 100000
    # self.rho_0 = 5
    self.k_att = k_att
    self.k_rep = k_rep
    self.rho_0 = rho_0
    
  def generate_force_vector(self, id, agent_list, object_list, world_radius):
  
    # Add attraction to goal
    force = self.attractionForce(agent_list[id])
    
    # Add repulstion from other agents
    for i in range(len(agent_list)):
      if i != id:
        force = self.add(force, self.repulsionForce(agent_list[id], agent_list[i], "agent"))
    
    # Add repulsion from objects
    for i in range(len(object_list)):
      force = self.add(force, self.repulsionForce(agent_list[id], object_list[i], "object"))

    # Check wall
    # x
    if np.abs(world_radius - agent_list[id].pose.x) < self.rho_0:        
      sign = np.sign(agent_list[id].pose.x)
      wall_object = Pose(sign*world_radius, agent_list[id].pose.y, 0)
      force = self.add(force, self.wallRepulsion(agent_list[id], wall_object))

    if np.abs(world_radius - agent_list[id].pose.y) < self.rho_0:
      sign = np.sign(agent_list[id].pose.y)
      wall_object = Pose(agent_list[id].pose.x, sign*world_radius, 0)
      force = self.add(force, self.wallRepulsion(agent_list[id], wall_object))
    
    F_net = self.normalize(force)

    return F_net

  def wallRepulsion(self, agent, wall_pose):

    dist = math.sqrt((agent.pose.x - wall_pose.x)**2 + (agent.pose.y - wall_pose.y)**2)

    k = self.k_rep*2

    F_x = k*(1/dist - 1/self.rho_0)*(agent.pose.x - wall_pose.x)/dist**3
    F_y = k*(1/dist - 1/self.rho_0)*(agent.pose.y - wall_pose.y)/dist**3

    return [F_x, F_y]
    #return [F_y, -F_x]
    

  def attractionForce(self, agent):
    
    # Edit this function
    #dist = self.euclid_dist(agent)

    #dist = sqrt((agent.goal_pose.x - agent.pose.x)**2 + (agent.goal_pose.y - agent.pose.y)**2)

    if True:#id == 0:
      F_x = -self.k_att*(agent.pose.x - agent.goal_pose.x)
      F_y = -self.k_att*(agent.pose.y - agent.goal_pose.y)
    # else:
    #   F_x = -self.k_att*(agent.pose.x - agent.goal_pose.x - 5)
    #   F_y = -self.k_att*(agent.pose.y - agent.goal_pose.y - 5)


    return [F_x, F_y]
  
  def repulsionForce(self, agent1, agent2, force_type):
    
    # Edit this function
    dist = math.sqrt((agent1.pose.x - agent2.pose.x)**2 + (agent1.pose.y - agent2.pose.y)**2)

    # if force_type == "agent":
    #   k = self.k_rep/2
    # else: 
    #   k = self.k_rep

    if force_type == 'agent' and self.id == 1:
      k = 0
    else:
      if force_type == 'agent':
        k = self.k_rep / 2
      else:
        k = self.k_rep

    F_x = 0
    F_y = 0

    if dist > self.rho_0:
      F_x = 0
      F_y = 0

    else:
      F_x = k*(1/dist - 1/self.rho_0)*(agent1.pose.x - agent2.pose.x)/dist**3
      F_y = k*(1/dist - 1/self.rho_0)*(agent1.pose.y - agent2.pose.y)/dist**3

    # if self.id == 1:
    #   print('Fx', F_x, 'Fy', F_y)
    
    # if force_type == 'agent':
    #   return [F_x, F_y] 

    return [F_x, F_y]
    #return [F_y, -F_x]
    
  def add(self, force_1, force_2):
    return [force_1[0] + force_2[0], force_1[1] + force_2[1]]
    
  def normalize(self, vector):
    # length = math.sqrt(vector[0]**2 + vector[1]**2)
    # return [vector[0] / length, vector[1] / length]
    return vector


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

    if id == 0:
      self.af = AgentForce(id)
      self.v_max = 1.75
    else:
      self.af = AgentForce(id, k_att = 10, k_rep = 1000000, rho_0=4)
      self.v_max = 2.25 # This is for collision between pursuer and evader
      # self.v_max = 2 # This should run it into the wall
    
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
    # if self.id == 0:
    #   return self.af0.generate_force_vector(self.id, agent_list, object_list, world_radius)
    # if self.id == 1:
    #   return self.af1.generate_force_vector(self.id, agent_list, object_list, world_radius)
    return self.af.generate_force_vector(self.id, agent_list, object_list, world_radius)

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
  
  # def collision(self, agent):
  #   """ Function to check if the agent is in collision with another agent.
  #       Arguments:
  #         agent (APF_agent): The other agent there could be a collision with.
  #       Returns:
  #         collision (Bool): True if there is a collision.
  #   """
  #   dist = math.sqrt((self.pose.x - agent.pose.x)**2 + (self.pose.y - agent.pose.y)**2)
  #   return dist < self.radius + agent.radius and self.alive and agent.alive
    
  # def remove(self):
  #   """ Function to reset the pose to be at the origin and set alive to false.
  #   """
  #   self.pose = Pose(0, 0, 0)
  #   self.alive = False
    
  # def out_of_bounds(self, world_edge):
  #   """ Function to check if the agent is out of bounds.
  #       Arguments:
  #         world_radius (float): The radius of the world in m.
  #       Returns:
  #         out_of_bounds (Bool): True if the agent is out of bounds.
  #   """
  #   #dist = math.sqrt(self.pose.x**2 + self.pose.y**2)
  #   x = self.pose.x
  #   y = self.pose.y
  #   r = self.radius

  #   out = x < world_edge

  #   return dist > world_radius - self.radius
    
  def at_goal(self):
    """ Function to check if the agent has reached its goal.
        Returns:
          at_goal (Bool): True if the agent is at its goal pose.
    """
    # print('Distance to goal: ', self.pose.distance_to(self.goal_pose))
    return self.pose.distance_to(self.goal_pose) < goal_threshold

  # def update_score(self, time_step):
  #   """ Function to update an agents score based on goal reached and collisions.
  #       Arguments:
  #         time_step (float): The world's current time step in s.
  #   """
  #   if self.alive:
  #     if self.at_goal():
  #       self.score = initial_score - time_step
  #       self.remove()
  #       print("Agent ",self.id,"done with score",self.score)
  #     elif self.collided:
  #       self.score = time_step
  #       self.remove()
  #       print("Agent ",self.id,"done with score",self.score)
