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

    #find the closest point in time
    current_time = current_state[0]
    traj_times = np.array([traj_point[0] for traj_point in self.traj])
    closest_idx = np.argmin(np.abs(traj_times - current_time))

    closest_point = self.traj[closest_idx]

    delta_t = LOOK_AHEAD_TIME


    if(closest_idx != len(self.traj) - 1):

      next_closest_point = self.traj[closest_idx + 1]

      #calculate the speed of the robot between the two points
      delta_x = next_closest_point[1] - closest_point[1])
      delta_y = next_closest_point[2] - closest_point[2])
      speed = np.sqrt(np.square(delta_x) + np.square(delta_y)) / (next_closest_point[0] - closest_point[0])

      #calculate the slope of the line between the two points
      slope = delta_y / delta_x

      #linearly interpolate the point we should set as the destination
      #these equations were found by solving the system of equations for x and y that satisified
      #(1) being a distance away from the starting point of v*delta_t
      #(2) being on the line formed between the two x, y pairs
      x_dest = speed*delta_t/(np.sqrt(np.square(slope) + 1)) + closest_point[1]
      y_dest = speed*delta_t*slope/(np.sqrt(np.square(slope) + 1)) + closest_point[2]

      #calculating the angular interpolation
      angular_speed = (next_closest_point[3] - closest_point[3]) / (next_closest_point[0] - closest_point[0])

      theta_dest = closest_point[3] + angular_speed*delta_t

      time_dest = closest_point[0] + delta_t

      destination = [time_dest, x_dest, y_dest, theta_dest]

    else:

      destination = self.traj[-1]

    return destination
  
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
    k_rho = 9
    k_alpha = 10
    k_beta = -7

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
