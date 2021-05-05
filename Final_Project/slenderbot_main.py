# Import helper files

import math
import matplotlib.pyplot as plt
import numpy as np
from agent import *
from utilities import *
import copy

def create_world(world_edge, num_obj, obj_rad, agent_list, epsilon = 4):

  #keep track of our successful object creations
  created_obstacles = 0
  obj_list = []

  while created_obstacles < num_obj:

    if created_obstacles % 4 == 0: # Bottom left quad
      x_obj = np.random.uniform(-world_edge + obj_rad, 0)
      y_obj = np.random.uniform(-world_edge + obj_rad, 0)
    elif created_obstacles % 4 == 1: # Bottom right
      x_obj = np.random.uniform(0, world_edge-obj_rad)
      y_obj = np.random.uniform(-world_edge + obj_rad, 0)
    elif created_obstacles % 4 == 2: # Top right
      x_obj = np.random.uniform(0, world_edge-obj_rad)
      y_obj = np.random.uniform(0, world_edge-obj_rad)
    else:
      x_obj = np.random.uniform(-world_edge + obj_rad, 0)
      y_obj = np.random.uniform(0, world_edge-obj_rad)

    
    # x_obj = np.random.uniform(-world_edge + obj_rad, world_edge-obj_rad)
    # y_obj = np.random.uniform(-world_edge + obj_rad, world_edge-obj_rad)


    skip_iteraton = False
    for agent in agent_list + obj_list:
      
      start = agent.pose 
      goal = agent.goal_pose

      dist_to_start = np.sqrt(np.square(x_obj - start.x) + np.square(y_obj - start.y))
      dist_to_goal = np.sqrt(np.square(x_obj - goal.x) + np.square(y_obj - goal.y))

      if(dist_to_start <= obj_rad + agent.radius + epsilon or dist_to_goal <= obj_rad + agent.radius + epsilon):
        skip_iteraton = True

    if(skip_iteraton == False):
      pose = Pose(x_obj, y_obj, 0)
      obj = Agent(False, pose, pose, obj_rad, -1)
      obj_list.append(obj)
      created_obstacles += 1
        
  return obj_list
  

def checkStatus(agent_list, obj_list):

  for agent in agent_list:
    if agent.at_goal(agent_list):
      return ('W', agent.id)

    for obj in obj_list:
      if agent.collision(obj):
        print(agent.id, "collided")
        return ('L', agent.id)

    # for otherAgent in agent_list:
    #   if otherAgent.id != agent.id:
    #     if agent.collision(otherAgent):
    #       print(agent.id, "collided")
    #       return ('L', agent.id)

    # if agent.collision():
    #   print(agent.id, "collided")
    if agent.out_of_bounds(25):
      print(agent.id, "out of bounds")
      return ('L', agent.id)
      
      # TODO: RETURN DEAD
  return None

if __name__ == '__main__':

  # Create world
  print("hi slenderbo—OH SHIT")

  test_pose = Pose(20, -20, np.pi/2)#Pose(0, 0, 0)
  test_pose2 = Pose(-20, -20, 0)
  #test_pose2 = Pose(-2, 4, 0) #all other tests
  # test_pose2 = Pose(-12, -5, 0) # Let evader win
  goal_pose = Pose(0, 20, 0)#Pose(15, 15, 0)
  robot_radius = 1
  evader = Agent(False, test_pose, goal_pose, robot_radius, 0, "EXP")

  pursuer = Agent(False, test_pose2, evader.pose, robot_radius, 1, "EXP")
  agent_list = [evader, pursuer]

  #objects are LoL x, y, radius
  obj1_pose = Pose(5, 5, 0)
  obj2_pose = Pose(-5, -5, 0)
  obstacle_radius = 1.5 #4
  world_edge = 25
  obj_list = create_world(world_edge, 16, obstacle_radius, agent_list)#[Agent(False, obj1_pose, obj1_pose, obstacle_radius, -1), Agent(False, obj2_pose, obj2_pose, obstacle_radius, -1)]

  #initialize any expansive planners
  agent_count = 0
  for agent in agent_list:
    if(agent.plannerType != "APF"):
      agent.exp_planner.init_traj(agent, obj_list, world_edge, agent_count, agent_list)


  #define variables of interest
  
  #obj_list = [[5, 5, 1], [-5, -5, 1]]
  #obj_list = [Pose(5, 5, 0), Pose(-5, -5, 0)]
  

  #so its the same type of stand as Star Platinum
  #so its the same type of plot as plot_world
  plot_za_warudo(agent_list, obj_list, world_edge, True, evader_goal=goal_pose)

  experiment_running = True
  delta_t = 0.1
  time_step = 0
  max_time_step = 100
  incremental_theta = 0
  traj_evader = []
  traj_pursuer = []

  while time_step < max_time_step and experiment_running:

    # test_agent.pose.x = 10*np.sin(2*np.pi*1/max_time_step*time_step)
    # test_agent.pose.theta = incremental_theta
    # incremental_theta += delta_t*2*np.pi/max_time_step

    for agent in agent_list:
      agent.update(time_step, delta_t, agent_list, obj_list, world_edge)
      if agent.id == 0:
        traj_evader.append([time_step, agent.pose.x, agent.pose.y, agent.pose. theta])

      if agent.id == 1:
        traj_pursuer.append([time_step, agent.pose.x, agent.pose.y, agent.pose. theta])
      # print("agent state: ", "[", agent.pose.x, ", ", agent.pose.y, ", ", agent.pose.theta, "]" )
      # print("APF planner state: ", "[", agent.APF_planner.pose.x, ", ", agent.APF_planner.pose.y, ", ", agent.APF_planner.pose.theta, "]" )

    #plot_za_warudo([test_agent], [[5, 5, 1], [-5, -5, 1]], 25, False)
    plot_za_warudo(agent_list, obj_list, world_edge, False, evader_goal=goal_pose)

    status = checkStatus(agent_list, obj_list)

    if status is not None:
      if status[0] == 'W':
        print('Agent ', status[1], " has won")
      if status[0] == 'L':
        print('Agent ', status[1], " has lost")
      plot_za_warudo(agent_list, obj_list, world_edge, True)
      experiment_running = False

    #agent_list[1].updateGoalPose(agent_list[0].pose)

    time_step += delta_t

  plot_overall_trajectory(agent_list, obj_list, 25, traj_evader, traj_pursuer)


#TODO:
#1: Fix bug w/ traj merging in Expansive planner x
#2: Tune APFs (go fest wtf)
#3: Tune repulsion especially
#4: tune expansive planner (plan time budget, etc)
#5: figure out good experiment settings
#6: different update rates for evader/pursuer
  


  
  