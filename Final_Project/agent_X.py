# E206 Motion Planning

# AFP planner
# C Clark

import math
import matplotlib.pyplot as plt
from APF_utilities import *

class AgentForce_X():
  
  def __init__(self):
    self.id = X
    
  def generate_force_vector(self, id, agent_list, object_list, world_radius):
  
    # Add attraction to goal
    force = self.attractionForce(agent_list[id])
    
    # Add repulstion from other agents
    for i in range(len(agent_list)):
      if i != id:
        force = self.add(force, self.repulsionForce(agent_list[id], agent_list[i]))
    
    # Add repulsion from objects
    for i in range(len(object_list)):
      force = self.add(force, self.repulsionForce(agent_list[id], object_list[i]))
    
    return self.normalize(force)

  def attractionForce(self, agent):
    
    # Edit this function
    return [1, 0]
  
  def repulsionForce(self, agent1, agent2):
    
    # Edit this function
    return [1, 0]
    
  def add(self, force_1, force_2):
    return [force_1[0] + force_2[0], force_1[1] + force_2[1]]
    
  def normalize(self, vector):
    length = math.sqrt(vector[0]**2 + vector[1]**2)
    return [vector[0] / length, vector[1] / length]
