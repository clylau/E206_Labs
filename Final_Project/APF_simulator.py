# E206 Motion Planning

# AFP planner
# C Clark

import math
import matplotlib.pyplot as plt
import numpy as np
from APF_agent import *
from APF_utilities import *

# Constants
delta_t = 0.2 #s

def create_world(num_agents):
  """ Function to create a simulated world of agents.
      Arguments:
        num_agents (Int): The number of agents to populate the world with.
      Returns:
        agent_list (list of APF_agents): The autonomous agents in the world.
        object_list (list of APF_agents): The objects in the world that agents should avoid.
        world_radius (float): The radius the world's workspace
  """
  world_radius = 5 + 2 * num_agents
  zero_pose = Pose(0,0,0)
  object_list = [APFAgent(pose=zero_pose, goal_pose=zero_pose, radius=2, id=0)]
  delta_angle = 2*math.pi/num_agents
  
  agent_list = []
  for i in range(num_agents):
    pose_angle = i*delta_angle
    pose_opposite_angle= angle_diff(pose_angle + math.pi)
    pose_radius = world_radius - 1
    pose = Pose(pose_radius*math.cos(pose_angle), pose_radius*math.sin(pose_angle), pose_opposite_angle)
    goal_pose = Pose(pose_radius*math.cos(pose_opposite_angle), pose_radius*math.sin(pose_opposite_angle), pose_opposite_angle)
    agent = APFAgent(pose, goal_pose, default_agent_radius, i)
    
    agent_list.append(agent)
    
  return agent_list, object_list, world_radius
  
def update_agent_scores(agent_list, time_step):
  """ Function to update the scores of all agents in the world.
      Arguments:
        agent_list (list of APF_agents): The autonomous agents in the world.
      Returns:
        agent_list (list of APF_agents): The autonomous agents in the world.
  """
  for i in range(len(agent_list)):
    agent_list[i].update_score(time_step)
        
  return agent_list
  
def some_agents_alive(agent_list):
  """ Function to check if any of the agents are alive.
      Arguments:
        agent_list (list of APF_agents): The autonomous agents in the world.
      Returns:
        some_agents_alive (Bool): True if any agent is still alive.
  """
  for agent in agent_list:
    if agent.alive:
      return True
  
  return False

def check_for_collisions(agent_list, object_list, world_radius):
  """ Function to check if the agent is out of bounds.
      Arguments:
        agent_list (list of APF_agents): The autonomous agents in the world.
        object_list (list of APF_agents): The objects in the world that agents should avoid.
        world_radius (float): The radius the world's workspace
      Returns:
        agent_list (list of APF_agents): The autonomous agents in the world.
  """
  for i in range(len(agent_list)):
    if agent_list[i].out_of_bounds(world_radius):
      agent_list[i].collided = True
    else:
      for j in range(len(object_list)):
        if agent_list[i].collision(object_list[j]):
          agent_list[i].collided = True
      for j in range(i+1,len(agent_list)):
        if agent_list[i].collision(agent_list[j]):
          agent_list[i].collided = True
          agent_list[j].collided = True
  
  return agent_list

if __name__ == '__main__':

  # Create world
  num_agents = 10
  agent_list, object_list, world_radius = create_world(num_agents = num_agents)
  agent_scores = [initial_score] * num_agents
  last_agent_list = agent_list
  plot_world(agent_list, object_list, world_radius, True)

  # Loop over time
  max_time_steps = 100
  time_step = 0
  while time_step < max_time_steps and some_agents_alive(agent_list):
    
    # update all agents
    for agent in agent_list:
      agent.update(delta_t, last_agent_list, object_list, world_radius)
    
    # check for collisions
    agent_list = check_for_collisions(agent_list, object_list, world_radius)

    # calculate scores
    agent_list = update_agent_scores(agent_list, time_step)
    
    # plot
    plot_world(agent_list, object_list, world_radius, False)
    
    # remember last list of objects
    last_agent_list = agent_list
    
    # update time
    time_step += delta_t

  # Print final scores
  print("\n\nFinal Scores")
  print("==================================================")
  for i in range(len(agent_list)):
    print("Agent",i,"'s score is",agent_list[i].score)
