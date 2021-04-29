# Import helper files

import math
import matplotlib.pyplot as plt
import numpy as np
from agent import *
from utilities import *

def create_world():


  return None

def checkStatus(agent_list, obj_list):

  for agent in agent_list:
    if agent.at_goal():
      return agent.id
  return None

if __name__ == '__main__':

  # Create world
  print("hi slenderbo—OH SHIT")

  test_pose = Pose(0, 0, 0)
  test_pose2 = Pose(-2, 4, 0)
  goal_pose = Pose(15, 15, 0)
  test_agent = Agent(False, test_pose, goal_pose, 1, 0, "APF")

  test_agent2 = Agent(False, test_pose2, test_agent.pose, 1, 1, "APF")


  #objects are LoL x, y, radius
  obj1_pose = Pose(5, 5, 0)
  obj2_pose = Pose(-5, -5, 0)
  obj_list = [Agent(False, obj1_pose, obj1_pose, 1, -1), Agent(False, obj2_pose, obj2_pose, 1, -1)]

  #define variables of interest
  agent_list = [test_agent, test_agent2]
  #obj_list = [[5, 5, 1], [-5, -5, 1]]
  #obj_list = [Pose(5, 5, 0), Pose(-5, -5, 0)]
  world_edge = 25

  #so its the same type of stand as Star Platinum
  #so its the same type of plot as plot_world
  plot_za_warudo(agent_list, obj_list, world_edge, True)

  experiment_running = True
  delta_t = 0.1
  time_step = 0
  max_time_step = 25
  incremental_theta = 0

  while time_step < max_time_step and experiment_running:

    # test_agent.pose.x = 10*np.sin(2*np.pi*1/max_time_step*time_step)
    # test_agent.pose.theta = incremental_theta
    # incremental_theta += delta_t*2*np.pi/max_time_step

    for agent in agent_list:
      agent.update(delta_t, agent_list, obj_list, world_edge)
      # print("agent state: ", "[", agent.pose.x, ", ", agent.pose.y, ", ", agent.pose.theta, "]" )
      # print("APF planner state: ", "[", agent.APF_planner.pose.x, ", ", agent.APF_planner.pose.y, ", ", agent.APF_planner.pose.theta, "]" )

    #plot_za_warudo([test_agent], [[5, 5, 1], [-5, -5, 1]], 25, False)
    plot_za_warudo(agent_list, obj_list, world_edge, False)

    status = checkStatus(agent_list, obj_list)

    if status is not None:
      print('Agent ', status, " has won")
      plot_za_warudo(agent_list, obj_list, world_edge, True)
      experiment_running = False

    #agent_list[1].updateGoalPose(agent_list[0].pose)

    time_step += delta_t


#TODO:
#1: Make a decision how to store information (pose, list, etc.) and be consistent about it
#2: Organize the class structure framework in a way that makes sense
#3: Do a goal check
#4: do a collision check
#5: get two robots in the environment, one of the robots should chase the other
#6: Tune the APF so the robots move fest
#7: get a haircut
#8: Cry
#0: graDUATE 
  


  
  