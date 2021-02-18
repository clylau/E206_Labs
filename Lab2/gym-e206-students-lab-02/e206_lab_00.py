import gym
import gym_fetch
import time
import math
import random
from traj_planner_utils import *
from traj_tracker import *

    
def main():
  # Create a motion planning problem and solve it
  current_state, desired_state, objects, walls = create_motion_planning_problem()
  desired_traj = construct_dubins_traj(current_state, desired_state)
  
  # Construct an environment
  env = gym.make("fetch-v0") # <-- this we need to create
  env.set_parameters(TIME_STEP_SIZE, objects)
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
      print("Cur:",current_state,"Des:",desired_state)
      action = controller.point_tracking_control(desired_state, current_state)
      observation, reward, done, dummy = env.step(action)
      env.render('human')
      actual_traj.append(current_state)
      current_time_stamp += TIME_STEP_SIZE
  time.sleep(2)
  plot_traj(desired_traj, actual_traj, objects, walls)
  
  env.close()
  
def create_motion_planning_problem():
  current_state = [0, 0, 0, 0]
  desired_state = [20, 5.0, 2.0, 0]
  maxR = 8
  walls = [[-maxR, maxR, maxR, maxR, 2*maxR], [maxR, maxR, maxR, -maxR, 2*maxR], [maxR, -maxR, -maxR, -maxR, 2*maxR], [-maxR, -maxR, -maxR, maxR, 2*maxR] ]
  objects = [[4, 0, 1.0], [-2, -3, 1.5]]
  
  return current_state, desired_state, objects, walls

if __name__ == '__main__':
    main()
    
    
