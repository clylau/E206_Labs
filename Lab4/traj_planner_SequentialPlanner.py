# E206 Motion Planning

# Simple planner
# C Clark

import math
import dubins
import random
import matplotlib.pyplot as plt
from traj_planner_utils import *
import numpy as np
import time
from traj_planner_ExpPlanner import *

class Planning_Problem():
  """ Class that holds a single motion planning problem for one robot.
  """

  def __init__(self, initial_state, desired_state, objects, walls):
    """ Constructor for planning problem.
        - Arguments:
          - initial_state (list of floats): The initial state of the robot (t, x, y, theta).
          - desired_state (list of floats): The desired state of the robot (t, x, y, theta)
          - objects (list of list of floats): The objects to avoid.
          - walls (list of list of floats): The walls to not drive through.
    """
    self.initial_state = initial_state
    self.desired_state = desired_state
    self.objects = objects
    self.walls = walls
    self.planner = Expansive_Planner()
    
  def add_trajs_as_obstacles(self, traj_set):
    """ Function to add trajs as obstacles to the pp.
        - Arguments
          - traj_set (list of list of floats): The trajectories to be treated as obstacles.
    """
    for traj in traj_set:
      self.objects.append(['traj',traj])

class Sequential_Planner():
  
  def __init__(self):
    pass

  def construct_traj_set(self, planning_problem_list):
    """ Function that creates a set of trajectories for several planning problems (1 per robot).
        - Parameters:
          - planning_problem_list (list of Planning_Problems): One pp to solve per robot.
        - Returns:
          - traj_set (list of list of floats): One traj per robot.
    """
    
    # Add code here to create a traj set #

    otherTrajectories = []
    traj_set = []

    start_time = time.time()
    for i in range(len(planning_problem_list)):

      planProb = planning_problem_list[i]
      expPlanner = planProb.planner # Get expansive planner

      # Add old trajectories as obstacles first
      planProb.add_trajs_as_obstacles(otherTrajectories)

      # Create desired traj
      desired_traj, _ = expPlanner.construct_traj(planProb.initial_state, planProb.desired_state, planProb.objects, planProb.walls)
      # desired_traj, _ = expPlanner.construct_optimized_traj(planProb.initial_state, planProb.desired_state, planProb.objects, planProb.walls)

      # Add trajectory to list of old trajectories
      if len(desired_traj) != 0:
        otherTrajectories.append(desired_traj)

      traj_set.append(desired_traj)
      # print("num obstacles: ", len(planProb.objects))

    total_time = time.time() - start_time
    mean_time = total_time / len(planning_problem_list)
    print("num planning problems: ", len(planning_problem_list))
    print("num found trajs: ", len(otherTrajectories))
    if(len(planning_problem_list) != len(otherTrajectories)):
      mean_time = np.NaN
    #print(mean_time)
        
    return traj_set, mean_time
      
def random_pose(maxR):
  """ Function to generate random pose (x, y, theta) within bounds +/- maxR.
      Parameters:
        - maxR (float): the half width of the square environment in meters.
      Returns:
        - random_pose (list of floats): The x, y, theta in (m, m, rad).
  """
  return [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), random.uniform(-math.pi, math.pi)]
  
def get_new_random_pose(pose_list, maxR, radius):
  """ Function to generate random pose (x, y, theta) within bounds +/- maxR.
      Parameters:
        - pose_list (list of floats):
        - maxR (float): the half width of the square environment in meters.
        - radius (float): The objects' radius in meters.
      Returns:
        - random_pose (list of floats): The x, y, theta, radius in (m, m, rad, m).
  """
  new_pose_found = False
  while not new_pose_found:
    new_pose = random_pose(maxR)
    new_pose_found = True
    for pl in pose_list:
      effective_radius = radius + pl[3]
      if (abs(pl[0]-new_pose[1]) < effective_radius and abs(pl[1]-new_pose[2]) < effective_radius) or (abs(pl[0]-new_pose[1]) < effective_radius and abs(pl[1]-new_pose[2]) < effective_radius):
        new_state_found = False
  
  return new_pose + [radius]


if __name__ == '__main__':
  disable_plotting = True
  numTrials = 100
  maxRobots = 5
  num_objects = 0
  maxR = 10
  obj_vel = 0
  plan_time = 5 #seconds

  total_means = np.zeros(numTrials)
  for trial in range(numTrials):
    for robots in range(1):
      num_robots = maxRobots
      robot_initial_pose_list = []
      robot_initial_state_list = []

      for _ in range(num_robots):
        ns = get_new_random_pose(robot_initial_pose_list, maxR, ROBOT_RADIUS)
        robot_initial_pose_list += [ns]
        robot_initial_state_list += [[0, ns[0], ns[1], ns[2]]]

        # #Hardcoded Dynamic obstacles test
        # robot_initial_state_list += [[0, 0, -9, np.pi/2]]
      
      robot_desired_pose_list = []
      robot_desired_state_list = []
      for _ in range(num_robots):
        ns = get_new_random_pose(robot_desired_pose_list, maxR, ROBOT_RADIUS)
        robot_desired_pose_list += [ns]
        robot_desired_state_list += [[120, ns[0], ns[1], ns[2]]]

        # #Hardcoded Dynamic obstacles test
        # robot_desired_state_list += [[120, 0, 9, np.pi/2]]

      object_list = []
      for b in range(num_objects):
        object_radius = random.uniform(0.3, 1.0)
        obj_pose = get_new_random_pose(robot_initial_pose_list + robot_desired_pose_list, maxR, object_radius)
        obj_yaw = obj_pose[2]
        object_list += [ ['obstacle',[obj_pose[0], obj_pose[1], 0.5, obj_vel, obj_yaw]] ]

        # #Hardcoded Dynamic obstacles test
        # object_list += [ ['obstacle',[-2, -7+2*b, 0.5, obj_vel, 0.0]] ]
        
      walls = [[-maxR, maxR, maxR, maxR, 2*maxR], [maxR, maxR, maxR, -maxR, 2*maxR], [maxR, -maxR, -maxR, -maxR, 2*maxR], [-maxR, -maxR, -maxR, maxR, 2*maxR] ]
        
      planning_problem_list = []
      for i in range(num_robots):
        pp = Planning_Problem(robot_initial_state_list[i], robot_desired_state_list[i], object_list, walls)
        planning_problem_list.append(pp)
      
      test_means = []
      start_time = time.time()
      while time.time() - start_time <= plan_time:

        for idx in range(len(planning_problem_list)):
          planning_problem_list[idx].objects = object_list

        planner = Sequential_Planner()
        traj_list, mean_time = planner.construct_traj_set(planning_problem_list)
        # print("num Objects in problem: ", len(planning_problem_list[0].objects))
        test_means.append(mean_time)

      
      #add the time to the numpy array
      total_means[trial] = np.nanmean(test_means)

      if (len(traj_list) > 0) and not disable_plotting:
        plot_traj_list(traj_list, object_list, walls)
        #print(traj_list)

  # robot_mean_plan_times = np.nanmean(total_means, axis = 1)
  # plt.plot(np.arange(1, maxRobots + 1), robot_mean_plan_times)
  # plt.xlabel("Number of Robots (#)")
  # plt.ylabel("Mean Plan Time Construction (s)")
  # plt.show()

  # print("Mean Plan time for 5 robots: ", robot_mean_plan_times[3])


# if __name__ == '__main2__':
#   num_robots = 4
#   num_objects = 2
#   maxR = 10
#   obj_vel = 0.0
  
#   robot_initial_pose_list = []
#   robot_initial_state_list = []
#   for _ in range(num_robots):
#     ns = get_new_random_pose(robot_initial_pose_list, maxR, ROBOT_RADIUS)
#     robot_initial_pose_list += [ns]
#     robot_initial_state_list += [[0, ns[0], ns[1], ns[2]]]
  
#   robot_desired_pose_list = []
#   robot_desired_state_list = []
#   for _ in range(num_robots):
#     ns = get_new_random_pose(robot_desired_pose_list, maxR, ROBOT_RADIUS)
#     robot_desired_pose_list += [ns]
#     robot_desired_state_list += [[0, ns[0], ns[1], ns[2]]]

#   object_list = []
#   for _ in range(num_objects):
#     object_radius = random.uniform(0.3, 1.0)
#     obj_pose = get_new_random_pose(robot_initial_pose_list + robot_desired_pose_list, maxR, object_radius)
#     obj_yaw = obj_pose[2]
#     object_list += [ ['obstacle',[obj_pose[0], obj_pose[1], 0.5, obj_vel, obj_yaw]] ]
    
#   walls = [[-maxR, maxR, maxR, maxR, 2*maxR], [maxR, maxR, maxR, -maxR, 2*maxR], [maxR, -maxR, -maxR, -maxR, 2*maxR], [-maxR, -maxR, -maxR, maxR, 2*maxR] ]
    
#   planning_problem_list = []
#   for i in range(num_robots):
#     pp = Planning_Problem(robot_initial_state_list[i], robot_desired_state_list[i], object_list, walls)
#     planning_problem_list.append(pp)
  
#   planner = Sequential_Planner()
#   traj_list, _ = planner.construct_traj_set(planning_problem_list)
#   if len(traj_list) > 0:
#     plot_traj_list(traj_list, object_list, walls)
