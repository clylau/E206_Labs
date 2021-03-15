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

class Node():

  def __init__(self, state, parent_node, edge_distance):
    self.state = state
    self.parent_node = parent_node
    self.edge_distance = edge_distance
    
  def manhattan_distance_to_node(self, node):
    return abs(self.state[1] - node.state[1]) + abs(self.state[2] - node.state[2])
  
  def manhattan_distance_to_state(self, state):
    return abs(self.state[1] - state[1]) + abs(self.state[2] - state[2])
    
  def euclidean_distance_to_state(self, state):
    return math.sqrt( (self.state[1] - state[1])**2 + (self.state[2] - state[2])**2 )

class Expansive_Planner():
  
  DISTANCE_DELTA = 1.5 #m
  DIST_TO_GOAL_THRESHOLD = 0.5 #m
  GRID_RESOLUTION = 0.5 #m
  EDGE_TIME = 10 #s
  LARGE_NUMBER = 9999999
  
  MAX_NUM_ITERATIONS = 1000
  MIN_RAND_DISTANCE = 1.0 #m
  MAX_RAND_DISTANCE = 5.0 #m
  MEAN_EDGE_VELOCITY = 0.75 #m
  PLAN_TIME_BUDGET = 0.5 #s
    
  def __init__(self):
    self.fringe = []

  def adjust_variables(self, min_dist, max_dist):
    """
    """
    self.MIN_RAND_DISTANCE = min_dist
    self.MAX_RAND_DISTANCE = max_dist
  
  def construct_traj(self, initial_state, desired_state, objects, walls):
    """ Construct a trajectory in the X-Y space and in the time-X,Y,Theta space.
        Arguments:
          traj_point_0 (list of floats): The trajectory's first trajectory point with time, X, Y, Theta (s, m, m, rad).
          traj_point_1 (list of floats): The trajectory's last trajectory point with time, X, Y, Theta (s, m, m, rad).
        Returns:
          traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
          traj_cost (float): The path length (m).
    """
    self.tree = []
    self.desired_state = desired_state
    self.objects = objects
    self.walls = walls
    
    # Add code here to make a traj #
    initialNode = Node(initial_state, None, 0)
    self.add_to_tree(initialNode)

    notDone = True
    count = 0
    goalNode = None

    while(notDone and count < self.MAX_NUM_ITERATIONS):
      count = count + 1
      randNode = self.sample_random_node()
      newNode = self.generate_random_node(randNode)

      if (newNode.edge_distance < self.LARGE_NUMBER):
        self.add_to_tree(newNode)

        goalNode = self.generate_goal_node(newNode, desired_state)

        if goalNode is not None:
          notDone = False

    traj = []
    traj_cost = self.LARGE_NUMBER
      
    if(goalNode is not None):
      traj, traj_cost = self.build_traj(goalNode)
      
    return traj, traj_cost
    
  def construct_optimized_traj(self, initial_state, desired_state, objects, walls, returnCounts = False):
    """ Construct the best trajectory possible within a limited time budget.
        Arguments:
          traj_point_0 (list of floats): The trajectory's first trajectory point with time, X, Y, Theta (s, m, m, rad).
          traj_point_1 (list of floats): The trajectory's last trajectory point with time, X, Y, Theta (s, m, m, rad).
        Returns:
          best_traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
          best_traj_cost (float): The path lenght of the shortest traj (m).
    """
    best_traj = []
    best_traj_cost = self.LARGE_NUMBER
    start_time = time.perf_counter()
    numIters = 0
    numSuccesses = 0
    total_costs = 0

    while((time.perf_counter() - start_time) < self.PLAN_TIME_BUDGET):

      currTraj, currCost = self.construct_traj(initial_state, desired_state, objects, walls)

      if(currCost < self.LARGE_NUMBER):
        total_costs = total_costs + currCost
        numSuccesses = numSuccesses + 1

      if(currCost < best_traj_cost):
        best_traj = currTraj
        best_traj_cost = currCost

      numIters = numIters + 1
    
    if(returnCounts):
      print("Total Successes: ",numSuccesses ,"Number of Tries: ", numIters)
      print("Average (successful) path length: ", total_costs/(numSuccesses + 1e-9))
      print()

    return best_traj, best_traj_cost
    
  def add_to_tree(self, node):
    """ Add the node to the tree.
        Arguments:
          node (Node): The node to be added.
    """
    
    # Add code here to add a node to the tree#

    self.tree.append(node)
    
  def sample_random_node(self):
    """ Randomly select a node from the tree and return it.
        Returns:
          node (Node): A randomly selected node from the tree.
    """
    
    # Add code here to return a random node from the tree #

    # Using naive approach because dum
    idx = np.arange(len(self.tree))

    randomIdx = np.random.choice(idx, 1)[0]

    return self.tree[randomIdx]   
    
  def generate_random_node(self, node_to_expand):
    """ Create a new node by expanding from the parent node using.
        Arguments:
          node_to_expand (Node): The parent node.
        Returns:
          new_node (Node): The newly generated node.
    """
    
    # Add code here to make a new node #

    rand_dist = np.random.uniform(self.MIN_RAND_DISTANCE, self.MAX_RAND_DISTANCE)
    rand_angle = np.random.uniform(0, 2*np.pi)

    t = node_to_expand.state[0]
    x = node_to_expand.state[1]
    y = node_to_expand.state[2]
    theta = node_to_expand.state[3]

    t_child = t + rand_dist / self.MEAN_EDGE_VELOCITY
    x_child = x + rand_dist * np.cos(angle_diff(theta + rand_angle))
    y_child = y + rand_dist * np.sin(angle_diff(theta + rand_angle))
    theta_child = angle_diff(theta + 2*rand_angle)

    edge_dist = self.calculate_edge_distance([t_child, x_child, y_child, theta_child], node_to_expand)

    childNode = Node([t_child, x_child, y_child, theta_child], node_to_expand, edge_dist)

    return childNode

  def generate_goal_node(self, node, desired_state):
    """ Create a goal node by connecting from the parent node using.
        Arguments:
          node_to_expand: The parent node.
        Returns:
          goal_node: The newly generated goal node or None if there is not goal connection.
    """
    
    # Add code here to make a goal node if possible #

    #This function checks if the node of interest has a direct path to the goal. If so, we return the goal node
    #if not, we return None
    edge_dist = self.calculate_edge_distance(desired_state, node)

    if(edge_dist >= self.LARGE_NUMBER):
      return None

    goal_node = Node(desired_state, node, edge_dist)

    #otherwise, we've found our path!
    return goal_node
      

  def calculate_edge_distance(self, state, parent_node):
    """ Calculate the cost of an dubins path edge from a parent node's state to another state.
        Arguments:
          state: The end state of the edge.
          parent_node: The initial state node of the edge.
        Returns:
          traj_distance: The length of the edge, or is the LARGE_NUMBER if collision exists (m).
    """
    traj, traj_distance = construct_dubins_traj(parent_node.state, state)
    if collision_found(traj, self.objects, self.walls):
      return self.LARGE_NUMBER

    return traj_distance

  def build_traj(self, goal_node):
    """ Build a traj via back tracking from a goal node.
        Arguments:
          goal_node: The node to back track from and create a traj of dubins paths with.
        Returns:
          traj (list of list of floats): The trajectory as a list of time, X, Y, Theta (s, m, m, rad).
          traj_cost (float): The length of the traj (m).
    """
    node_list = []
    node_to_add = goal_node
    while node_to_add != None:
      node_list.insert(0, node_to_add)
      node_to_add = node_to_add.parent_node
  
    traj = []
    traj_cost = 0
    for i in range(1,len(node_list)):
      node_A = node_list[i-1]
      node_B = node_list[i]
      traj_point_0 = node_A.state
      traj_point_1 = node_B.state
      traj_point_1[3] = math.atan2(traj_point_1[2]-traj_point_0[2], traj_point_1[1]-traj_point_0[1])
      edge_traj, edge_traj_distance = construct_dubins_traj(traj_point_0, traj_point_1)
      traj = traj + edge_traj[:-1]
      traj_cost = traj_cost + edge_traj_distance
    
    return traj, traj_cost

  def collision_found(self, node_1, node_2):
    """ Return true if there is a collision with the traj between 2 nodes and the workspace
        Arguments:
          node_1 (Node): A node with the first state of the traj - Time, X, Y, Theta (s, m, m, rad).
          node_2 (Node): A node with the second state of the traj - Time, X, Y, Theta (s, m, m, rad).
          objects (list of lists): A list of object states - X, Y, radius (m, m, m).
          walls (list of lists): A list of walls defined by end points - X0, Y0, X1, Y1, length (m, m, m, m, m).
        Returns:
          collision_found (boolean): True if there is a collision.
    """
    traj, traj_distance = construct_dubins_traj(node_1.state, node_2.state)
    return collision_found(traj, self.objects, self.walls)


if __name__ == '__main__':

  total_path_cost = 0
  num_successes = 0

  #defining varialbes to save experiment data
  baseline_path_cost = []
  exp1_path_cost = []
  exp2_path_cost = []
  exp3_path_cost = []

  baseline_numSuccess = 0
  exp1_numSuccess = 0
  exp2_numSuccess = 0
  exp3_numSuccess = 0

  numTrials = 1000

  for i in range(0, numTrials):
    maxR = 10
    tp0 = [0, -8, -8, 0]
    tp1 = [300, 8, 8, 0]

    #define each of the different planners for use in the experiments
    #we need to run every time to reset the planner values for each trial
    planner_base = Expansive_Planner()

    planner_exp1 = Expansive_Planner()
    planner_exp1.adjust_variables(5, 9)

    planner_exp2 = Expansive_Planner()
    planner_exp2.adjust_variables(1, 2)

    planner_exp3 = Expansive_Planner()
    planner_exp3.adjust_variables(8, 9)

    #creates the walls and objects for this trial
    walls = [[-maxR, maxR, maxR, maxR, 2*maxR], [maxR, maxR, maxR, -maxR, 2*maxR], [maxR, -maxR, -maxR, -maxR, 2*maxR], [-maxR, -maxR, -maxR, maxR, 2*maxR] ]
    num_objects = 10
    objects = []
    for j in range(0, num_objects): 
      obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 1.0]
      while (abs(obj[0]-tp0[1]) < 1 and abs(obj[1]-tp0[2]) < 1) or (abs(obj[0]-tp1[1]) < 1 and abs(obj[1]-tp1[2]) < 1):
        obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 1.0]
      objects.append(obj)


    #traj, traj_cost = planner.construct_optimized_traj(tp0, tp1, objects, walls, returnCounts=True)
    #traj, traj_cost = planner.construct_traj(tp0, tp1, objects, walls)

    #####run all our planners####
    #run baseline planner
    base_traj, base_cost = planner_base.construct_traj(tp0, tp1, objects, walls)

    if(len(base_traj) > 0):
      baseline_numSuccess += 1
      baseline_path_cost.append(base_cost)

    #run experiment 1 planner
    exp1_traj, exp1_cost = planner_exp1.construct_traj(tp0, tp1, objects, walls)

    if(len(exp1_traj) > 0):
      exp1_numSuccess += 1
      exp1_path_cost.append(exp1_cost)

    #run experiment 2 planner
    exp2_traj, exp2_cost = planner_exp2.construct_traj(tp0, tp1, objects, walls)

    if(len(exp2_traj) > 0):
      exp2_numSuccess += 1
      exp2_path_cost.append(exp2_cost)

    #run experiment 3 planner
    exp3_traj, exp3_cost = planner_exp3.construct_traj(tp0, tp1, objects, walls)

    if(len(exp3_traj) > 0):
      exp3_numSuccess += 1
      exp3_path_cost.append(exp3_cost)



  #### calculate and printout results ######
  #baseline results
  base_mean = np.mean(baseline_path_cost) 
  base_std = np.std(baseline_path_cost)
  base_min = np.min(baseline_path_cost)
  base_max = np.max(baseline_path_cost)
  base_success_rate = baseline_numSuccess / numTrials

  print("Baseline Results")
  print("Number of trials: ", numTrials, " Minimum node distance: ", planner_base.MIN_RAND_DISTANCE, " Maximum node distance: ", planner_base.MAX_RAND_DISTANCE)
  print("Mean: ", base_mean," Stdev: ", base_std, " min: ", base_min, " Max: ", base_max)
  print("success rate: ", base_success_rate)
  print()

  #experiment 1 results
  exp1_mean = np.mean(exp1_path_cost) 
  exp1_std = np.std(exp1_path_cost)
  exp1_min = np.min(exp1_path_cost)
  exp1_max = np.max(exp1_path_cost)
  exp1_success_rate = exp1_numSuccess / numTrials

  print("Experiment 1 Results")
  print("Number of trials: ", numTrials, " Minimum node distance: ", planner_exp1.MIN_RAND_DISTANCE, " Maximum node distance: ", planner_exp1.MAX_RAND_DISTANCE)
  print("Mean: ", exp1_mean,"Stdev: ", exp1_std, "min: ", exp1_min, "Max: ", exp1_max)
  print("success rate: ", exp1_success_rate)
  print()

  #experiment 2 results
  exp2_mean = np.mean(exp2_path_cost) 
  exp2_std = np.std(exp2_path_cost)
  exp2_min = np.min(exp2_path_cost)
  exp2_max = np.max(exp2_path_cost)
  exp2_success_rate = exp2_numSuccess / numTrials

  print("Experiment 2 Results")
  print("Number of trials: ", numTrials, " Minimum node distance: ", planner_exp2.MIN_RAND_DISTANCE, " Maximum node distance: ", planner_exp2.MAX_RAND_DISTANCE)
  print("Mean: ", exp2_mean,"Stdev: ", exp2_std, "min: ", exp2_min, "Max: ", exp2_max)
  print("success rate: ", exp2_success_rate)
  print()

  #experiment 3 results
  exp3_mean = np.mean(exp3_path_cost) 
  exp3_std = np.std(exp3_path_cost)
  exp3_min = np.min(exp3_path_cost)
  exp3_max = np.max(exp3_path_cost)
  exp3_success_rate = exp3_numSuccess / numTrials

  print("Experiment 3 Results")
  print("Number of trials: ", numTrials, " Minimum node distance: ", planner_exp3.MIN_RAND_DISTANCE, " Maximum node distance: ", planner_exp3.MAX_RAND_DISTANCE)
  print("Mean: ", exp3_mean,"Stdev: ", exp3_std, "min: ", exp3_min, "Max: ", exp3_max)
  print("success rate: ", exp3_success_rate)
  print()

    #   total_path_cost += traj_cost


  # print("Average path length: ", total_path_cost/num_successes)
  # print("number of successful paths generated: ", num_successes, "out of 20 trials")
