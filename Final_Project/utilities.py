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

def plot_za_warudo(agent_list, object_list, world_edge_length, inital_plot):
    """
    """

    # fig = plt.figure()
    # ax = plt.gca()

    ang_res = 1#0.2
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
        r_obj = 1
        
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

    plt.axis("equal")
    if(inital_plot):
        plt.show()
    else:
        plt.pause(0.01)

    plt.cla()


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

    
        