# E206 Motion Planning

# Simple planner
# C Clark

import math
import dubins
import matplotlib.pyplot as plt
import numpy as np

DISTANCE_STEP_SIZE = 0.1 #m
COLLISION_INDEX_STEP_SIZE = 5

def construct_dubins_traj(traj_point_0, traj_point_1):
  """ Construc a trajectory in the X-Y space and in the time-X,Y,Theta space.
      Arguments:
        traj_point_0 (list of floats): The trajectory's first trajectory point with time, X, Y, Theta (s, m, m, rad).
        traj_point_1 (list of floats): The trajectory's last trajectory point with time, X, Y, Theta (s, m, m, rad).
      Returns:
        traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
  """

  traj = []
  traj_point = [0,0,0,0]
  traj.append(traj_point)

  turning_rad = 1.0
  step_size = DISTANCE_STEP_SIZE

  path = dubins.shortest_path(traj_point_0[1:], traj_point_1[1:], turning_rad)
  configs, _ = path.sample_many(step_size)

  dt = (traj_point_1[0] - traj_point_0[0]) / len(configs)
  timeStamp = traj_point_0[0]

  for i in range(len(configs)):
      configs[i] = list(configs[i])
      configs[i].insert(0, timeStamp)
      timeStamp += dt

  configs.append(traj_point_1)
  # print()
  # print(configs)

  # print()
  #print(traj_point_0)
  #print(traj_point_1)
  #print(configs)
      
  #return traj
  return configs

def plot_traj(traj_desired, traj_actual, objects, walls):
  """ Plot a trajectory in the X-Y space and in the time-X,Y,Theta space.
      Arguments:
        desired_traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
        actual_traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
        objects (list of lists): A list of stationay object states with X, Y, radius (m, m, m).
        walls (list of lists: A list of walls with corners X1, Y1 and X2, Y2 points, length (m, m, m, m, m).
  """
  fig, axis_array = plt.subplots(2,1)
  time_stamp_desired = []
  x_desired = []
  y_desired = []
  theta_desired = []
  for tp in traj_desired:
    time_stamp_desired.append(tp[0])
    x_desired.append(tp[1])
    y_desired.append(tp[2])
    theta_desired.append(angle_diff(tp[3]))
  axis_array[0].plot(x_desired, y_desired, 'b')
  axis_array[0].plot(x_desired[0], y_desired[0], 'ko')
  axis_array[0].plot(x_desired[-1], y_desired[-1], 'kx')
  time_stamp_actual = []
  x_actual = []
  y_actual = []
  theta_actual = []
  for tp in traj_actual:
    time_stamp_actual.append(tp[0])
    x_actual.append(tp[1])
    y_actual.append(tp[2])
    theta_actual.append(angle_diff(tp[3]))
  axis_array[0].plot(x_actual, y_actual, 'k')

  ang_res = 0.2
  for o in objects:
    x_obj = []
    y_obj = []
    ang = 0
    while ang < 6.28:
      x_obj.append(o[0]+o[2]*math.cos(ang))
      y_obj.append(o[0]+o[2]*math.sin(ang))
      ang += ang_res
    x_obj.append(x_obj[0])
    y_obj.append(y_obj[0])
    axis_array[0].plot(x_obj, y_obj, 'k')
  for w in walls:
    axis_array[0].plot([w[0], w[2]], [w[1], w[3]], 'k')
  axis_array[0].set_xlabel('X (m)')
  axis_array[0].set_ylabel('Y (m)')
  axis_array[0].axis('equal')
  
  axis_array[1].plot(time_stamp_desired, x_desired,'b')
  axis_array[1].plot(time_stamp_desired, y_desired,'b--')
  axis_array[1].plot(time_stamp_desired, theta_desired,'b-.')
  axis_array[1].plot(time_stamp_actual, x_actual,'k')
  axis_array[1].plot(time_stamp_actual, y_actual,'k--')
  axis_array[1].plot(time_stamp_actual, theta_actual,'k-.')
  axis_array[1].set_xlabel('Time (s)')
  axis_array[1].legend(['X Desired (m)', 'Y Desired (m)', 'Theta Desired (rad)', 'X (m)', 'Y (m)', 'Theta (rad)'])

  plt.show()
  
def collision_found(traj, objects, walls):
  """ Return true if there is a collision with the traj and the workspace
      Arguments:
        traj (list of lists): A list of traj points - Time, X, Y, Theta (s, m, m, rad).
        objects (list of lists): A list of object states - X, Y, radius (m, m, m).
        walls (list of lists): A list of walls defined by end points - X0, Y0, X1, Y1, length (m, m, m, m, m).
      Returns:
        collision_found (boolean): True if there is a collision.
  """

  return False
  
def generate_distance_to_object(traj_point, obj):
  """ Calculate the deistance between a spherical object and a cylindrical robot.
      Argument:
        traj_point (list of floats): A state of Time, X, Y, Theta (s, m, m, rad).
        obj (list of floats): An object state X, Y, radius (m, m, m).
      Returns:
        distance (float): The distance between a traj point and an object (m).
  """
  return math.sqrt( pow(traj_point[1]-obj[0],2) + pow(traj_point[2]-obj[1],2) )
  
def generate_distance_to_wall(traj_point, wall):
  """ Calculate the deistance between a spherical object and a cylindrical robot.
      Argument:
        traj_point (list of floats): A state of Time, X, Y, Theta (s, m, m, rad).
        wall (list of floats): An wall state X0, Y0, X1, Y1, length (m, m, m, m, m).
      Returns:
        distance (float): The distance between a traj point and an object (m).
  """
  
  return 0
  
def print_traj(traj):
  """ Print a trajectory as a list of traj points.
      Arguments:
        traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
  """
  print("TRAJECTORY")
  for tp in traj:
    print("traj point - time:",tp[0], "x:", tp[1], "y:", tp[2], "theta:", tp[3] )
    
def angle_diff(ang):
  """ Function to push ang within the range of -pi and pi
      Arguments:
        ang (float): An angle (rad).
      Returns:
        ang (float): The angle, but bounded within -pi and pi (rad).
  """
  while ang > math.pi:
    ang -= 2*math.pi
  while ang < -math.pi:
    ang += 2*math.pi

  return ang


def calculateErrors(desired_traj, actual_traj):
  """
  Yare Yare Daze
  ba buh bunoh bah 
  """

  #pick the closest points in time to the desired tracjectory
  closest_idxs = np.zeros(len(desired_traj), dtype = "int32")
  traj_times = np.array([traj_point[0] for traj_point in actual_traj])

  for i in range(len(desired_traj)):

    current_time = desired_traj[i][0]
    closest_idx = np.argmin(np.abs(traj_times - current_time))

    closest_idxs[i] = closest_idx

  
  closest_idxs = closest_idxs.astype("int32")
  actual_xs = np.array([traj_point[1] for traj_point in actual_traj])
  actual_ys = np.array([traj_point[2] for traj_point in actual_traj])
  actual_thetas = np.array([traj_point[3] for traj_point in actual_traj])

  x_actual = actual_xs[closest_idxs]
  y_actual = actual_ys[closest_idxs]
  theta_actual = actual_thetas[closest_idxs]

  x_desired = np.array([traj_point[1] for traj_point in desired_traj])
  y_desired = np.array([traj_point[2] for traj_point in desired_traj])
  theta_desired = np.array([traj_point[3] for traj_point in desired_traj])

  x_rms = np.sqrt(np.mean(np.square(x_actual - x_desired)))
  y_rms = np.sqrt(np.mean(np.square(y_actual - y_desired)))

  #calculate the theta differences individual because vectorizing is for losers fuck you
  theta_diffs = np.zeros(len(theta_desired))
  for i in range(len(theta_desired)):
    theta_diffs[i] = angle_diff(theta_actual[i] - theta_desired[i])

  theta_rms = np.sqrt(np.mean(np.square(theta_diffs)))

  return x_rms, y_rms, theta_rms

  
if __name__ == '__main__':
  #tp0 = [0,0,0,0]
  #tp0 = [0, -3, -3, -math.pi/4]
  #tp0 = [0, -3, 3, 0]
  tp0 = [0, -3, 0, math.pi/2]

  #tp1 = [5, 3, 3, math.pi/2]#[10,4,-4, -1.57]
  #tp1 = [10, 3, 3, -math.pi/4]
  #tp1 = [5, 3, -3, 0]
  tp1 = [10, 3, 0, -math.pi/2]

  traj = construct_dubins_traj(tp0, tp1)
  maxR = 8
  walls = [[-maxR, maxR, maxR, maxR], [maxR, maxR, maxR, -maxR], [maxR, -maxR, -maxR, -maxR], [-maxR, -maxR, -maxR, maxR] ]
  objects = [[4, 0, 1.0], [-2, -3, 1.5]]
  plot_traj(traj, traj, objects, walls)
