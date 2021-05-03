import numpy as np
import matplotlib.pyplot as plt
import math
import dubins

DISTANCE_STEP_SIZE = 0.1 #m
COLLISION_INDEX_STEP_SIZE = 5
ROBOT_RADIUS = 1 #m
BUFFER_TIME = 0.4 #s
COLLISION_BUFFER = 0.2 #m



class Pose():

    def __init__(self, x, y, theta):

        self.x = x
        self.y = y
        self.theta = theta

    def distance_to(self, pose):
        return math.sqrt((self.x - pose.x)**2 + (self.y - pose.y)**2)

def plot_za_warudo(agent_list, object_list, world_edge_length, inital_plot, evader_goal=None):
    """
    """

    # fig = plt.figure()
    # ax = plt.gca()

    ang_res = 0.6
    for agent in agent_list:

        #plot the agent circle
        x_center = agent.pose.x
        y_center = agent.pose.y
        rad = agent.radius

        ang = np.arange(0, 2*np.pi, ang_res, dtype = "float")

        x_agent = x_center + rad*np.cos(ang)
        y_agent = y_center + rad*np.sin(ang)

        plt.plot(x_agent, y_agent)

        #plot the agent pointer
        theta = agent.pose.theta

        x_pointer = [x_center, x_center + rad*np.cos(theta)]
        y_pointer = [y_center, y_center + rad*np.sin(theta)]
        plt.plot(x_pointer, y_pointer, "k")

        if(agent.plannerType != "APF"):
          if(len(agent.exp_planner.traj) != 0):
            traj = np.array(agent.exp_planner.traj)
            plt.plot(traj[:, 1], traj[:, 2], "r--")


    #plot the objects
    for obj in object_list:
        # x_center = obj[0]
        # y_center = obj[1]
        # r_obj = obj[2]

        x_center = obj.pose.x
        y_center = obj.pose.y
        r_obj = obj.radius
        
        ang = np.arange(0, 2*np.pi, step = ang_res, dtype = "float")

        #freindship ended with math and for loops, now np is my new best friend
        x_obj = x_center + r_obj*np.cos(ang) #fuck math.cos, all my homies hate math.cos
        y_obj = y_center + r_obj*np.sin(ang)

        x_obj = np.append(x_obj, x_obj[0])
        y_obj = np.append(y_obj, y_obj[0])

        plt.plot(x_obj, y_obj, "k")

    #plot bounds
    x_low = -world_edge_length
    x_high = world_edge_length
    y_low = -world_edge_length
    y_high = world_edge_length

    x_wall_pos = [x_low, x_low, x_high, x_high, x_low]
    y_wall_pos = [y_low, y_high, y_high, y_low, y_low]

    plt.plot(x_wall_pos, y_wall_pos, "k")

    # plot goal
    if evader_goal is None:
      plt.plot(15, 15, 'kx')
    else:
      plt.plot(evader_goal.x, evader_goal.y, 'kx')

    plt.axis("equal")
    if(inital_plot):
        plt.show()
    else:
        plt.pause(0.01)

    plt.cla()

def plot_overall_trajectory(agent_list, object_list, world_edge_length, traj_evader, traj_pursuer):
  plt.subplot(2, 1, 1)
  ang_res = 0.6
  for agent in agent_list:

      #plot the agent circle
      x_center = agent.pose.x
      y_center = agent.pose.y
      rad = agent.radius

      ang = np.arange(0, 2*np.pi, ang_res, dtype = "float")

      x_agent = x_center + rad*np.cos(ang)
      y_agent = y_center + rad*np.sin(ang)

      plt.plot(x_agent, y_agent)

      #plot the agent pointer
      theta = agent.pose.theta

      x_pointer = [x_center, x_center + rad*np.cos(theta)]
      y_pointer = [y_center, y_center + rad*np.sin(theta)]
      plt.plot(x_pointer, y_pointer, "k")

  #plot the objects
  for obj in object_list:
      # x_center = obj[0]
      # y_center = obj[1]
      # r_obj = obj[2]

      x_center = obj.pose.x
      y_center = obj.pose.y
      r_obj = obj.radius
      
      ang = np.arange(0, 2*np.pi, step = ang_res, dtype = "float")

      #freindship ended with math and for loops, now np is my new best friend
      x_obj = x_center + r_obj*np.cos(ang) #fuck math.cos, all my homies hate math.cos
      y_obj = y_center + r_obj*np.sin(ang)

      plt.plot(x_obj, y_obj, "k")

  #plot bounds
  x_low = -world_edge_length
  x_high = world_edge_length
  y_low = -world_edge_length
  y_high = world_edge_length

  x_wall_pos = [x_low, x_low, x_high, x_high, x_low]
  y_wall_pos = [y_low, y_high, y_high, y_low, y_low]

  plt.plot(x_wall_pos, y_wall_pos, "k")

  # plot trajectory
  x_evader = [data[1] for data in traj_evader]
  y_evader = [data[2] for data in traj_evader]
  theta_evader = [data[3] for data in traj_evader]
  ts_evader = [data[0] for data in traj_evader]

  x_pursuer = [data[1] for data in traj_pursuer]
  y_pursuer = [data[2] for data in traj_pursuer]
  theta_pursuer = [data[3] for data in traj_pursuer]
  ts_pursuer = [data[0] for data in traj_pursuer]

  #print(x_evader)

  plt.plot(x_evader, y_evader, "b")
  plt.plot(x_pursuer, y_pursuer, "orange")
  plt.axis("equal")
  plt.xlabel('X (m)')
  plt.ylabel('Y (m)')

  plt.subplot(2, 1, 2)

  plt.plot(ts_evader, x_evader, 'b')
  plt.plot(ts_evader, y_evader, 'b-.')
  plt.plot(ts_evader, theta_evader, 'b--')

  plt.plot(ts_pursuer, x_pursuer, 'r')
  plt.plot(ts_pursuer, y_pursuer, 'r-.')
  plt.plot(ts_pursuer, theta_pursuer, 'r--')
  plt.xlabel('Time (s)')
  plt.ylabel('Units Vary')

  plt.legend(['X (evader)', 'Y (evader)', 'Theta(evader)', 'X (pursuer)', 'Y (pursuer)', 'Theta (pursuer)'])


  plt.show()



#todo: rewrite this so its not stupid
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

def construct_dubins_traj(traj_point_0, traj_point_1):
  """ Construc a trajectory in the X-Y space and in the time-X,Y,Theta space.
      Arguments:
        traj_point_0 (list of floats): The trajectory's first trajectory point with time, X, Y, Theta (s, m, m, rad).
        traj_point_1 (list of floats): The trajectory's last trajectory point with time, X, Y, Theta (s, m, m, rad).
      Returns:
        traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
        traj_distance (float): The length ofthe trajectory (m).
  """
  q0 = (traj_point_0[1], traj_point_0[2], traj_point_0[3])
  q1 = (traj_point_1[1], traj_point_1[2], traj_point_1[3])
  turning_radius = 0.5
  
  path = dubins.shortest_path(q0, q1, turning_radius)
  configurations, distances = path.sample_many(DISTANCE_STEP_SIZE)
  
  traj_distance = distances[-1]
  traj_time = traj_point_1[0] - traj_point_0[0]
  time_step_size = traj_time/len(distances)
  
  traj = []
  traj_point_time = traj_point_0[0]
  for c in configurations:
    traj_point = [traj_point_time, c[0], c[1], c[2]]
    traj.append(traj_point)
    traj_point_time += time_step_size
      
  return traj, traj_distance

    
def collision_found(traj, objects, walls, agent_radius = ROBOT_RADIUS):
  """ Return true if there is a collision with the traj and the workspace
      Arguments:
        traj (list of lists): A list of traj points - Time, X, Y, Theta (s, m, m, rad).
        objects (list of lists): A list of object states - X, Y, radius (m, m, m).
        walls (list of lists): A list of walls defined by end points - X0, Y0, X1, Y1, length (m, m, m, m, m).
      Returns:
        collision_found (boolean): True if there is a collision.
  """
  #assuming objects is passed in as a list of agent objects. Converting to a LoL
  # print(objects)
  objects = [["obstacle", [agent.pose.x, agent.pose.y, agent.radius, 0, 0]] for agent in objects]

  index = 0
  while index < len(traj):
    traj_point = traj[index]
    for object in objects:
      if object[0] == 'obstacle':
        obj = object[1]
        obj_radius = obj[2]
        obj_vel = obj[3]
        obj_yaw = obj[4]
        time_stamp = traj_point[0]
        obj_x = obj[0] + obj_vel*math.cos(obj_yaw)*time_stamp
        obj_y = obj[1] + obj_vel*math.sin(obj_yaw)*time_stamp
        obj_distance = generate_distance_to_object(traj_point, [obj_x, obj_y]) - obj_radius - ROBOT_RADIUS - COLLISION_BUFFER
        if obj_distance < 0:
          return True
      elif object[0] == 'traj':
        if collision_with_traj(object[1], traj_point):
          return True
        
    # for wall in walls:
    #   # wall_distance = generate_distance_to_wall(traj_point, wall) - ROBOT_RADIUS - COLLISION_BUFFER
    #   # if wall_distance < 0:
    #   #   return True

    #updated wall boundary check
    x = traj_point[1]
    y = traj_point[2]

    if(np.abs(x) >= walls - agent_radius or np.abs(y) >= walls - agent_radius):
      return True

    index += COLLISION_INDEX_STEP_SIZE
  
  return False
  
def collision_with_traj(traj, traj_point):
  
  traj_point_time = traj_point[0]
  begin_time = traj[0][0]
  end_time = traj[-1][0]
  if traj_point_time < begin_time - BUFFER_TIME or traj_point_time > end_time + BUFFER_TIME:
    return False
  elif traj_point_time < begin_time:
    traj_point_at_time = traj[0]
  elif traj_point_time > end_time:
    traj_point_at_time = traj[-1]
  else:
    time_fraction = traj_point_time / (end_time - begin_time)
    index = int(time_fraction*len(traj))
    traj_point_at_time = traj[index]
  
  distance = generate_distance_to_traj_point(traj_point, traj_point_at_time) - 2*ROBOT_RADIUS - COLLISION_BUFFER
  if distance < 0:
    return True
  
  return False
  
def generate_distance_to_traj_point(traj_point_0, traj_point_1):
  """ Calculate the deistance between a spherical object and a cylindrical robot.
      Argument:
        traj_point_0 (list of floats): A state of Time, X, Y, Theta (s, m, m, rad).
        traj_point_1 (list of floats): A state of Time, X, Y, Theta (s, m, m, rad).
      Returns:
        distance (float): The distance between a traj point and an object (m).
  """
  return math.sqrt( pow(traj_point_0[1]-traj_point_1[1],2) + pow(traj_point_0[2]-traj_point_1[2],2) )
  
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
  x0 = traj_point[1]
  y0 = traj_point[2]
  x1 = wall[0]
  y1 = wall[1]
  x2 = wall[2]
  y2 = wall[3]
  num = abs( (x2-x1)*(y1-y0) - (x1-x0)*(y2-y1) )
  den = wall[4]
  
  return num/den
        