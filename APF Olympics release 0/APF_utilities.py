# E206 Motion Planning

# AFP planner
# C Clark

import math
import matplotlib.pyplot as plt

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
  
def plot_world(agent_list, object_list, world_radius, initial_plot):
  """ Function to plot the simulated world.
      Arguments:
        agent_list: The autonomous agents in the world.
        object_list: The objects in the world that agents should avoid.
        world_radius: The radius the world's workspace
        initial_plot: True if this is the first plot of a sim.
  """
  delta_angle = 0.2
  
  # Plot agents.
  for agent in agent_list:
    x_circle_points = []
    y_circle_points = []
    angle = 0
    while angle < math.pi * 2 + delta_angle:
      angle += delta_angle
      x_circle_points.append(agent.pose.x + agent.radius * math.cos(angle))
      y_circle_points.append(agent.pose.y + agent.radius * math.sin(angle))
    plt.plot(x_circle_points, y_circle_points)
  plt.legend(["agent 0","agent 1","agent 2","agent 3","agent 4","agent 5","agent 6","agent 7","agent 8","agent 9","agent 10"])
  
  # Plot agent goals
  if not initial_plot:
    for agent in agent_list:
      plt.plot([agent.goal_pose.x], [agent.goal_pose.y],"ko")
  
  # Plot agent direction pointers.
  for agent in agent_list:
    x_pointer = [agent.pose.x, agent.pose.x + agent.radius * math.cos(agent.pose.theta)]
    y_pointer = [agent.pose.y, agent.pose.y + agent.radius * math.sin(agent.pose.theta)]
    plt.plot(x_pointer, y_pointer, "k")
  
  # Plot boundary.
  x_circle_points = []
  y_circle_points = []
  angle = 0
  while angle < math.pi * 2 + delta_angle:
    angle += delta_angle
    x_circle_points.append(world_radius * math.cos(angle))
    y_circle_points.append(world_radius * math.sin(angle))
  plt.plot(x_circle_points, y_circle_points,"k")
  
  # Plot objects.
  for obj in object_list:
    x_circle_points = []
    y_circle_points = []
    angle = 0
    while angle < math.pi * 2 + delta_angle:
      angle += delta_angle
      x_circle_points.append(obj.pose.x + obj.radius * math.cos(angle))
      y_circle_points.append(obj.pose.y + obj.radius * math.sin(angle))
    plt.plot(x_circle_points, y_circle_points, "k")

  plt.axis('equal')
  if initial_plot:
    plt.show()
  else:
    plt.pause(0.01)
  plt.cla()
