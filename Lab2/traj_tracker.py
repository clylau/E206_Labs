import time
import math
import random
from traj_planner_utils import *
import numpy as np

TIME_STEP_SIZE = 0.01 #s
LOOK_AHEAD_TIME = 1.0 #s
MIN_DIST_TO_POINT = 0.1 #m
MIN_ANG_TO_POINT = 0.10 #rad

class TrajectoryTracker():
  """ A class to hold the functionality for tracking trajectories.
      Arguments:
        traj (list of lists): A list of traj points Time, X, Y, Theta (s, m, m, rad).
  """
  current_point_to_track = 0
  traj_tracked = False
  traj = []

  def __init__(self, traj):
    self.current_point_to_track = 0
    self.traj = traj
    self.traj_tracked = False
      
  def get_traj_point_to_track(self, current_state):
    """ Determine which point of the traj should be tracked at the current time.
        Arguments:
          current_state (list of floats): The current Time, X, Y, Theta (s, m, m, rad).
        Returns:
          desired_state (list of floats: The desired state to track - Time, X, Y, Theta (s, m, m, rad).
    """
    self.current_point_to_track = 0

    return self.traj[self.current_point_to_track]
  
  def print_traj(self):
    """ Print the trajectory points.
    """
    print("Traj:")
    for i in range(len(self.traj)):
        print(i,self.traj[i])
          
  def is_traj_tracked(self):
    """ Return true if the traj is tracked.
        Returns:
          traj_tracked (boolean): True if traj has been tracked.
    """
    return self.traj_tracked
    
class PointTracker():
  """ A class to determine actions (motor control signals) for driving a robot to a position.
  """
  def __init__(self):
    self.traj_tracked = False
    #pass

  def get_dummy_action(self, x_des, x):
    """ Return a dummy action for now
    """
    action = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
    return action

  def point_tracking_control(self, desired_state, current_state):
    """ Return the motor control signals as actions to drive a robot to a desired configuration
        Arguments:
          desired_state (list of floats): The desired Time, X, Y, Theta (s, m, m, rad).
          current_state (list of floats): The current Time, X, Y, Theta (s, m, m, rad).
    """
    # zero all of action
    action = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]


    #define control law coefficients
    k_rho = 5
    k_alpha = 10
    k_beta = -5

    delta_x = desired_state[1] - current_state[1]
    delta_y = desired_state[2] - current_state[2]
    delta_theta = angle_diff(desired_state[3] - current_state[3])
    
    rho = np.sqrt(np.square(delta_x) + np.square(delta_y))

    #if we are within tolerance of the desired configuration, don't move the bot
    if(rho < MIN_DIST_TO_POINT and delta_theta < MIN_ANG_TO_POINT):
      self.traj_tracked = True
      return action

    alpha = angle_diff(-current_state[3] + np.arctan2(delta_y, delta_x))
    beta = 0 #we're def good coders and not just doing this because we can't remember python variable scope
    velocity = 0 #fuck memory management. Go fuck yourself. 
    sign = 0

    #if abs(alpha) is larger than 90 degrees, we want to drive backwards
    if(np.abs(alpha) > np.pi/2):
      alpha = angle_diff(-current_state[3] + np.arctan2(-delta_y, -delta_x))
      beta = angle_diff(-current_state[3] - alpha - desired_state[3])
      velocity = k_rho*rho
      sign = -1

    else:
      beta = angle_diff(-current_state[3] - alpha + desired_state[3])
      velocity = -k_rho*rho
      sign = 1

    #absorving r, L, and other constants into K gains, keeping negative signs from conversion
    right_wheel_torque = (k_alpha*alpha + k_beta*beta) + k_rho*rho*sign
    left_wheel_torque = -k_alpha*alpha - k_beta*beta + k_rho*rho*sign

    action[0] = right_wheel_torque
    action[1] = left_wheel_torque
    
    return action
