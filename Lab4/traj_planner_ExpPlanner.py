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
  PLAN_TIME_BUDGET = 2.0 #s
    
  def __init__(self):
    self.fringe = []

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
      
    return [], self.LARGE_NUMBER
    
  def construct_optimized_traj(self, initial_state, desired_state, objects, walls):
    """ Construct the best trajectory possible within a limited time budget.
        Arguments:
          traj_point_0 (list of floats): The trajectory's first trajectory point with time, X, Y, Theta (s, m, m, rad).
          traj_point_1 (list of floats): The trajectory's last trajectory point with time, X, Y, Theta (s, m, m, rad).
        Returns:
          best_traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
          best_traj_cost (float): The path lenght of the shortest traj (m).
    """
    start_time = time.perf_counter()
    best_traj = []
    best_traj_cost = self.LARGE_NUMBER
    
    # Add code here to make many trajs within a time budget and return the best traj #
    # You will want to call construct_traj #
      
    return best_traj, best_traj_cost
    
  def add_to_tree(self, node):
    """ Add the node to the tree.
        Arguments:
          node (Node): The node to be added.
    """
    
    # Add code here to add a node to the tree#
    pass
    
  def sample_random_node(self):
    """ Randomly select a node from the tree and return it.
        Returns:
          node (Node): A randomly selected node from the tree.
    """
        
    # Add code here to return a random node from the tree #
    
    return None
    
  def generate_random_node(self, node_to_expand):
    """ Create a new node by expanding from the parent node using.
        Arguments:
          node_to_expand (Node): The parent node.
        Returns:
          new_node (Node): The newly generated node.
    """
    
    # Add code here to make a new node #
    
    return None

  def generate_goal_node(self, node, desired_state):
    """ Create a goal node by connecting from the parent node using.
        Arguments:
          node_to_expand: The parent node.
        Returns:
          goal_node: The newly generated goal node or None if there is not goal connection.
    """
    
    # Add code here to make a goal node if possible #
      
    return None

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
      edge_traj, edge_traj_distance = construct_dubins_traj(traj_point_0, traj_point_1)
      traj = traj + edge_traj
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
  for i in range(0, 5):
    maxR = 10
    tp0 = [0, -8, -8, 0]
    tp1 = [300, 8, 8, 0]
    planner = Expansive_Planner()
    walls = [[-maxR, maxR, maxR, maxR, 2*maxR], [maxR, maxR, maxR, -maxR, 2*maxR], [maxR, -maxR, -maxR, -maxR, 2*maxR], [-maxR, -maxR, -maxR, maxR, 2*maxR] ]
    num_objects = 10
    objects = []
    for j in range(0, num_objects): 
      obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 1.0]
      while (abs(obj[0]-tp0[1]) < 1 and abs(obj[1]-tp0[2]) < 1) or (abs(obj[0]-tp1[1]) < 1 and abs(obj[1]-tp1[2]) < 1):
        obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 1.0]
      objects.append(obj)
    traj, traj_cost = planner.construct_optimized_traj(tp0, tp1, objects, walls)
    if len(traj) > 0:
      plot_traj(traj, traj, objects, walls)
