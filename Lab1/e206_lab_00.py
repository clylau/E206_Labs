import gym
import gym_fetch
import time
import math
import random
from traj_planner_utils import *
from traj_tracker import *
import numpy as np

    
def main():
  # Create a motion planning problem and solve it
  current_state, desired_state, objects, walls = create_motion_planning_problem()
  desired_traj = construct_dubins_traj(current_state, desired_state)
  
  # Construct an environment
  env = gym.make("fetch-v0") # <-- this we need to create
  env.set_parameters(TIME_STEP_SIZE, [])
  env.render('human')
  env.reset()

  # Create the trajectory and tracking controller
  controller = PointTracker()
  traj_tracker = TrajectoryTracker(desired_traj)
      
  # Create the feedback loop
  time.sleep(1)
  current_time_stamp = 0
  observation = [0,0,0,0,0]
  actual_traj = []
  while not traj_tracker.is_traj_tracked():
      current_state = [current_time_stamp, observation[0], observation[1], observation[2]]
      desired_state = traj_tracker.get_traj_point_to_track(current_state)
      action = controller.point_tracking_control(desired_state, current_state)
      observation, reward, done, dummy = env.step(action)
      env.render('human')
      actual_traj.append(current_state)
      current_time_stamp += TIME_STEP_SIZE
  time.sleep(2)
  plot_traj(desired_traj, actual_traj, [], [])
  x_rms, y_rms, theta_rms = calculateErrors(desired_traj, actual_traj)
  print("\n\n\n\n")
  print("x_rms: ", x_rms)
  print("y_rms: ", y_rms)
  print("theta_rms: ", theta_rms)
  print()

  env.close()
  
def main2():
  # Create a motion planning problem and solve it
  current_state, desired_state, objects, walls = create_motion_planning_problem()
  desired_traj = desired_state
  
  # Construct an environment
  env = gym.make("fetch-v0") # <-- this we need to create
  env.set_parameters(TIME_STEP_SIZE, [])
  env.render('human')
  env.reset()

  # Create the trajectory and tracking controller
  controller = PointTracker()
  traj_tracker = TrajectoryTracker(desired_traj)

  keep_tracking = True
  index = 0
  len_traj = len(desired_traj)
  
  # Create the feedback loop
  time.sleep(1)
  current_time_stamp = 0
  observation = [0,0,0,0,0]
  actual_traj = []

  while keep_tracking:
    current_state = [current_time_stamp, observation[0], observation[1], observation[2]]

    desired_state = desired_traj[index]
    # print(desired_state)

    action = controller.point_tracking_control(desired_state, current_state)
    observation, reward, done, dummy = env.step(action)
    env.render('human')
    actual_traj.append(current_state)
    current_time_stamp += TIME_STEP_SIZE

    if controller.traj_tracked:
      index = index + 1
      controller.traj_tracked = False

      if index == len_traj:
        keep_tracking = False

  time.sleep(2)
  plot_traj(desired_traj, actual_traj, [], [])
  env.close()

def create_motion_planning_problem():
  current_state = [0, 0, 0, 0]


  #point tracking experiment
  #desired_state = [[0, 2, 0, 0],[0, 0, 0, 0], [0, 2, 2, 0], [0, 0, 0, 0], [0, 0, 2, np.pi/2], [0, 0, 0, 0], [0, -2, 2, 0], [0, 0, 0, 0], [0, -2, 0, 0], [0, 0, 0, 0], [0, -2, -2, 0], [0, 0, 0, 0], [0, 0, -2, -np.pi/2], [0, 0, 0, 0], [0, 2, -2, 0], [0, 0, 0, 0],]

  
  #dubins desired states
  #desired_state = [20, 5, 0, 0]
  desired_state = [20, 5, 3, 0]
  #desired_state = [20, 5, -3, 0]

  #desired_state = [0, -2, 0, 0] #[2, 2, 0, 0] #current_state #[20, 2.5, -2.5, 1.57]
  maxR = 8
  walls = [[-maxR, maxR, maxR, maxR, 2*maxR], [maxR, maxR, maxR, -maxR, 2*maxR], [maxR, -maxR, -maxR, -maxR, 2*maxR], [-maxR, -maxR, -maxR, maxR, 2*maxR] ]
  objects = [[4, 0, 1.0], [-2, -3, 1.5]]
  
  return current_state, desired_state, objects, walls

if __name__ == '__main__':
    main()
    #main2()
    
    
