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

  def __init__(self, state, parent_node, g_cost, h_cost):
    self.state = state
    self.parent_node = parent_node
    self.g_cost = g_cost
    self.h_cost = h_cost
    self.f_cost = self.g_cost + self.h_cost
    
  def manhattan_distance_to_node(self, node):
    return abs(self.state[1] - node.state[1]) + abs(self.state[2] - node.state[2])
  
  def manhattan_distance_to_state(self, state):
    return abs(self.state[1] - state[1]) + abs(self.state[2] - state[2])
    
  def euclidean_distance_to_state(self, state):
    return math.sqrt( (self.state[1] - state[1])**2 + (self.state[2] - state[2])**2 )

class A_Star_Planner():
  
  DIST_TO_GOAL_THRESHOLD = 0.5 #m
  CHILDREN_DELTAS = [-0.5, -0.25, 0.0, 0.25, 0.5]
  DISTANCE_DELTA = 1.5 #m
  EDGE_TIME = 10 #s
  LARGE_NUMBER = 9999999


  def __init__(self):
    self.fringe = []

  def construct_traj(self, initial_state, desired_state, objects, walls):
    """ Construct a trajectory in the X-Y space and in the time-X,Y,Theta space.
        Arguments:
          traj_point_0 (list of floats): The trajectory's first trajectory point with time, X, Y, Theta (s, m, m, rad).
          traj_point_1 (list of floats): The trajectory's last trajectory point with time, X, Y, Theta (s, m, m, rad).
        Returns:
          traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
    """

    curTime = time.time()

    self.fringe = []
    self.desired_state = desired_state
    self.objects = objects #x, y coordinates of the objects
    self.walls = walls

    #generate the starting node
    self.add_to_fringe(self.create_initial_node(initial_state))

    path = []

    #while there are still nodes in the fringe
    while(len(self.fringe) != 0):

      #pop the best node off the fringe
      curr_node = self.get_best_node_on_fringe()

      goal_node = self.generate_goal_node(curr_node, desired_state)

      #if we have a clear path to the goal, go for it
      if(goal_node != None):
        break

      child_nodes = self.get_children(curr_node)

      self.fringe += child_nodes

    path = self.createPath(goal_node)

    #initialize the trajectory with the first point
    traj = []
    final_length = 0

    for idx in range(len(path) - 1):
      pose1 = path[idx]
      pose2 = path[idx + 1]
      
      inter_pose_traj, traj_edge_distance_list = construct_dubins_traj(pose1.state, pose2.state)
      
      #remove the start point from the trajectory to prevent doubling up (end point of last trajectory is start point of this one)
      if(idx != 0):
        inter_pose_traj = inter_pose_traj[1:]
      traj += inter_pose_traj

      final_length += traj_edge_distance_list[-1]

    timeTook = time.time()  - curTime
    
    return traj, final_length, timeTook
    
  def createPath(self, goal_node):
    """
    creates path from the goal node
    """
    currNode = goal_node
    path = []
    
    while(currNode.parent_node != None):
      path.insert(0, currNode)
      currNode = currNode.parent_node
    
    #insert the final node
    path.insert(0, currNode)

    return path

  def add_to_fringe(self, node):
    
    self.fringe.append(node)

    return

    
  def get_best_node_on_fringe(self):

    f_cost_list = [point.f_cost for point in self.fringe]

    # TODO if things break, add tie breaker using g_cost

    minPoint = np.argmin(f_cost_list)
    #print(f_cost_list[minPoint])

    return self.fringe.pop(minPoint)
    
  def get_children(self, node_to_expand):
    children_list = []
    t, x, y, theta = node_to_expand.state
    #print(t, x, y, theta)
    
    # Add code here.

    for i in range(len(self.CHILDREN_DELTAS)):
      t_c = t + self.EDGE_TIME
      x_c = x + self.DISTANCE_DELTA * np.cos(angle_diff(theta + self.CHILDREN_DELTAS[i]))
      y_c = y + self.DISTANCE_DELTA * np.sin(angle_diff(theta + self.CHILDREN_DELTAS[i]))
      theta_c = angle_diff(theta + 2*self.CHILDREN_DELTAS[i])

      childNode = self.create_node([t_c, x_c, y_c, theta_c], node_to_expand)

      children_list.append(childNode)
    
        
    return children_list

  def generate_goal_node(self, node, desired_state):
    
    #This function checks if the node of interest has a direct path to the goal. If so, we return the goal node
    #if not, we return None
    goal_node = self.create_node(desired_state, node)

    #if the trajectory between the goal and this node has a collison, return None
    if(self.collision_found(node, goal_node)):
      return None

    #otherwise, we've found our path!
    return goal_node
    
  def create_node(self, state, parent_node):
    
    # Add code here.
    
    # TODO DO NOT KEEP THIS FIX IT IDIOT D:
    g_cost = parent_node.g_cost + self.calculate_edge_distance(state, parent_node)
    h_cost = self.estimate_cost_to_goal(state)

    newNode = Node(state, parent_node, g_cost, h_cost)
    
    return newNode #Node(state, parent_node, g_cost, h_cost)

  def create_initial_node(self, state):
    
    # Add code here.
    g_cost = 0
    h_cost = self.estimate_cost_to_goal(state)

    return Node(state, None, g_cost, h_cost)

  def calculate_edge_distance(self, state, parent_node):
    
    #retrieve the parent state
    parent_state = parent_node.state

    #calculate the trajectory between the parent and new node
    traj, traj_distance = construct_dubins_traj(parent_state, state)

    if collision_found(traj, self.objects, self.walls):
      return self.LARGE_NUMBER

    #the edge cost is equal to the 
    edge_distance = traj_distance[-1]

    return edge_distance

  def estimate_cost_to_goal(self, state):
    return math.sqrt( (self.desired_state[1] - state[1])**2 + (self.desired_state[2] - state[2])**2 )

  def build_traj(self, goal_node):
    
    node_list = []
    node_to_add = goal_node
    while node_to_add != None:
      node_list.insert(0, node_to_add)
      node_to_add = node_to_add.parent_node
  
    traj = []
    for i in range(1,len(node_list)):
      node_A = node_list[i-1]
      node_B = node_list[i]
      traj_point_0 = node_A.state
      traj_point_1 = node_B.state
      traj_point_1[3] = math.atan2(traj_point_1[2]-traj_point_0[2], traj_point_1[1]-traj_point_0[1])
      edge_traj, edge_traj_distance = construct_dubins_traj(traj_point_0, traj_point_1)
      traj = traj + edge_traj
    
    return traj

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
    #return collision_found(traj, self.objects, self.walls)
    traj = np.array(traj)

    #radius of objects in environment
    r_check = 1.25

    #define the indices of the trajectory points to check for colisions
    traj_idx = np.arange(len(traj), step = 2)

    #placea all objects in a np array
    objects = np.array(self.objects)
    walls = np.array(self.walls)

    points_of_interest = traj[traj_idx]

    #check for collisions with the walls
    #note: assming the walls are square. in the future, if they are not, use this algorithm:
    #https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
    #https://stackoverflow.com/questions/3838329/how-can-i-check-if-two-segments-intersect?fbclid=IwAR2uBhTU-B7BFqL44hT6g9BfAIIpoZ4CD1LlSDb-eLUnk5TMAbUkUMF_xxw
    # def ccw(A,B,C):
    # return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)

    # # Return true if line segments AB and CD intersect
    # def intersect(A,B,C,D):
    #     return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

    x_walls = walls[:, [0, 2]]
    y_walls = walls[:, [1, 3]]

    x_lower = np.min(x_walls)
    y_lower = np.min(y_walls)
    x_upper = np.max(x_walls)
    y_upper = np.max(y_walls)

    x_points = points_of_interest[:, 1]
    y_points = points_of_interest[:, 2]

    #if any points are outside of the walls, there must have been a colision.
    if np.any(x_points > x_upper) or np.any(x_points < x_lower) or np.any(y_points > y_upper) or np.any(y_points < y_lower):
      return True


    x_objects = objects[:, 0]
    y_objects = objects[:, 1]

    for point in points_of_interest:

      x_point = point[1]
      y_point = point[2]
      
      distances = np.sqrt(np.square(x_point - x_objects) + np.square(y_point - y_objects))

      if np.any(distances < r_check):
        return True

    return False

if __name__ == '__main__':

  for i in range(0, 5):
    maxR = 10
    tp0 = [0, -8, -8, 0]
    tp1 = [300, random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0]
    planner = A_Star_Planner()
    walls = [[-maxR, maxR, maxR, maxR, 2*maxR], [maxR, maxR, maxR, -maxR, 2*maxR], [maxR, -maxR, -maxR, -maxR, 2*maxR], [-maxR, -maxR, -maxR, maxR, 2*maxR] ]
    num_objects = 25
    objects = []
    for j in range(0, num_objects): 
      obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0.5]
      while (abs(obj[0]-tp0[1]) < 1 and abs(obj[1]-tp0[2]) < 1) or (abs(obj[0]-tp1[1]) < 1 and abs(obj[1]-tp1[2]) < 1):
        obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0.5]
      objects.append(obj)
    traj, finalLength, timeTook = planner.construct_traj(tp0, tp1, objects, walls)
    if len(traj) > 0:
      plot_traj(traj, traj, objects, walls)

