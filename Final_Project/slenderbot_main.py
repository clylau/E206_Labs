# Import helper files

import math
import matplotlib.pyplot as plt
import numpy as np
from agent import *
from utilities import *
import copy

def create_world(world_edge, num_obj, obj_rad, agent_list, epsilon = 5):

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
        # print(agent.id, "collided")
        return ('L', agent.id)

    # for otherAgent in agent_list:
    #   if otherAgent.id != agent.id:
    #     if agent.collision(otherAgent):
    #       print(agent.id, "collided")
    #       return ('L', agent.id)

    # if agent.collision():
    #   print(agent.id, "collided")
    if agent.out_of_bounds(25):
      # print(agent.id, "out of bounds")
      return ('L', agent.id)
      
      # TODO: RETURN DEAD
  return None

if __name__ == '__main__':
  print("hi slenderbo—OH SHIT")

  #define constants
  evader_start = Pose(20, -20, np.pi/2)
  pursuer_start = Pose(-20, -20, 0)
  evader_goal = Pose(-10, 20, 0)
  robot_radius = 1
  obstacle_radius = 1.25 #4
  world_edge = 25

  #define information relevant to the experiment
  numTrials = 250
  evader_planner_types = ["APF", "APF", "EXP", "EXP"]
  pursuer_planner_types = ["APF", "EXP", "APF", "EXP"]
  assert(len(evader_planner_types) ==  len(pursuer_planner_types))
  enable_plotting = False

  #define variables to hold information about the results
  average_distances = np.zeros([len(evader_planner_types), numTrials])
  completion_times = np.zeros([2, len(evader_planner_types), numTrials]) #robotID x experiment x trial
  numFailures = np.zeros(len(evader_planner_types))
  numCollisions_evader = np.zeros(len(evader_planner_types))
  numCollisions_pursuer = np.zeros(len(evader_planner_types))
  numWins_evader = np.zeros(len(evader_planner_types))
  numWins_pursuer = np.zeros(len(evader_planner_types))

  #TODO: increment the trial counter after all 4 experiments have run
  trial_count = 0
  while(trial_count < numTrials):

    for experiment in range(len(evader_planner_types)):
      evader = Agent(False, copy.deepcopy(evader_start), copy.deepcopy(evader_goal), robot_radius, 0, evader_planner_types[experiment])
      pursuer = Agent(False, copy.deepcopy(pursuer_start), evader.pose, robot_radius, 1, pursuer_planner_types[experiment])
      agent_list = [evader, pursuer]

      #all four experiments are done on the same set of obstacles for the trial
      if(experiment == 0):
        obj_list = create_world(world_edge, 12, obstacle_radius, agent_list)

      #initialize any expansive planners
      agent_count = 0
      for agent in agent_list:
        if(agent.plannerType != "APF"):
          agent.exp_planner.init_traj(agent, obj_list, world_edge, agent_count, agent_list)
      

      #so its the same type of stand as Star Platinum
      #so its the same type of plot as plot_world
      if(enable_plotting):
        plot_za_warudo(agent_list, obj_list, world_edge, True, evader_goal=evader_goal)

      experiment_running = True
      delta_t = 0.1
      time_step = 0
      max_time_step = 100
      incremental_theta = 0
      traj_evader = []
      traj_pursuer = []

      while time_step < max_time_step and experiment_running:

        for agent in agent_list:
          agent.update(time_step, delta_t, agent_list, obj_list, world_edge)

          if agent.id == 0:
            traj_evader.append([time_step, agent.pose.x, agent.pose.y, agent.pose. theta])

          if agent.id == 1:
            traj_pursuer.append([time_step, agent.pose.x, agent.pose.y, agent.pose. theta])
          # print("agent state: ", "[", agent.pose.x, ", ", agent.pose.y, ", ", agent.pose.theta, "]" )
          # print("APF planner state: ", "[", agent.APF_planner.pose.x, ", ", agent.APF_planner.pose.y, ", ", agent.APF_planner.pose.theta, "]" )

        #plot_za_warudo([test_agent], [[5, 5, 1], [-5, -5, 1]], 25, False)
        if(enable_plotting):
          plot_za_warudo(agent_list, obj_list, world_edge, False, evader_goal=evader_goal)

        status = checkStatus(agent_list, obj_list)

        if status is not None:
          if status[0] == 'W':
            #print('Agent ', status[1], " has won")
            average_distances[experiment, trial_count] = average_distance(traj_pursuer, traj_evader)
            completion_times[0, experiment, trial_count] = np.NaN
            completion_times[1, experiment, trial_count] = np.NaN

            if(status[1] == 0):
              completion_times[0, experiment, trial_count] = traj_evader[-1][0]
              numWins_evader[experiment] += 1
            else:
              completion_times[1, experiment, trial_count] = traj_evader[-1][0]
              numWins_pursuer[experiment] += 1
            
          if status[0] == 'L':
            #print('Agent ', status[1], " has lost")
            #if the evader ran into an obstruction
            if(status[1] == 0):
              numCollisions_evader[experiment] += 1
              numFailures[experiment] += 1
              average_distances[experiment, trial_count] = np.NaN
            
            #if the pursuer ran into an obstruction
            else:
              numCollisions_pursuer[experiment] += 1
              numFailures[experiment] += 1
              average_distances[experiment, trial_count] = np.NaN

          if(enable_plotting):
            plot_za_warudo(agent_list, obj_list, world_edge, True)

          experiment_running = False

        time_step += delta_t

        #check if  the loop is about to end without a succesful completion
        if(time_step >= max_time_step):
          average_distances[experiment, trial_count] = np.NaN
          completion_times[0, experiment, trial_count] = np.NaN
          completion_times[1, experiment, trial_count] = np.NaN
          numFailures[experiment] += 1


      if(enable_plotting):
        plot_overall_trajectory(agent_list, obj_list, 25, traj_evader, traj_pursuer)

    #increment the trial count
    trial_count += 1
  
  #print out summary statistics

  for idx in range(len(pursuer_planner_types)):
    
    
    avg_dist_btwn = np.nanmean(average_distances[idx, :])
    avg_evad_time = np.nanmean(completion_times[0, idx, :])
    avg_pur_time = np.nanmean(completion_times[1, idx, :])
    exp_failure_rate = numFailures[idx]/numTrials
    evad_collision_rate = numCollisions_evader[idx]/numTrials
    pur_collision_rate = numCollisions_pursuer[idx]/numTrials
    evad_win_rate = numWins_evader[idx]/numTrials
    pur_win_rate = numWins_pursuer[idx]/numTrials

    print("Experiment ", idx, ": Evader - ", evader_planner_types[idx], "; Pursuer - ", pursuer_planner_types[idx])
    print("Average distance between robots: ", avg_dist_btwn)
    print("Average evader completion time: ", avg_evad_time)
    print("Average pursuer completion time: ", avg_pur_time)
    print("Evader win rate: ", evad_win_rate)
    print("Pursuer win rate: ", pur_win_rate)
    print("Experiment failure rate: ", exp_failure_rate)
    print("Evader obstruction collision rate: ", evad_collision_rate)
    print("Pursuer obstruction collision rate: ", pur_collision_rate)
    print()



#TODO:
#1: Fix bug w/ traj merging in Expansive planner x
#2: Tune APFs (go fest wtf)
#3: Tune repulsion especially
#4: tune expansive planner (plan time budget, etc)
#5: figure out good experiment settings
#6: different update rates for evader/pursuer
  


  
  