import numpy as np
import matplotlib.pyplot as plt
import math


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

    # plot goal
    if evader_goal is None:
      plt.plot(15, 15, 'kx')
    else:
      plt.plot(evader_goal.x, evader_goal.y)

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

    
        