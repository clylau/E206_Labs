import math
import matplotlib.pyplot as plt
from utilities import *
from APF import *
#from ExpansivePlanner import *

# Constants


class Agent():

    def __init__(self, isPursuer, pose, goal_pose, radius, id, plannerType=None):

        # Save room for both planners, later specify which one is not None
        self.APF_planner = None#APFAgent(pose, goal_pose, radius, id)
        self.exp_planner = None

        # Keep track of it
        self.plannerType = plannerType

        # Boolean to tell whether or not we're the pursuer
        # Will always be false if we're not a robot
        self.isPursuer = isPursuer
        self.id = id

        # Keep track of pose, if pursuer, ignore goal pose
        self.pose = pose
        self.goal_pose = goal_pose

        self.radius = radius

        # Planner Type if a robot
        if plannerType is not None:
            self.setPlanner(plannerType)
    
    def setPlanner(self, plannerType):

        if self.plannerType == 'APF':
            self.APF_planner = APFAgent(self.pose, self.goal_pose, self.radius, self.id)

        # else:
        #     self.exp_planner = ExpansivePlanner()
        return None

    def updateGoalPose(self, new_goal):
        self.goal_pose = new_goal

    def update(self, delta_t, agent_list, obj_list, world_edge):
        if self.id == 0:
            print('Pose of Evader: ', self.pose.x, self.pose.y, self.pose.theta)
        if self.id == 1:
            print('Goal of Pursuer: ', self.goal_pose.x, self.goal_pose.y, self.goal_pose.theta)

        if self.plannerType == 'APF':
            return self.APF_planner.update(delta_t, agent_list, obj_list, world_edge)

        # else:
        #     return self.exp_planner.getNextPose(time_stamp)

        # if self.id == 1:
        #     self.updateGoalPose(agent_list[0].pose)

        return None

    def collision(self, agent):
        # if self.plannerType == 'APF':
        #     return self.APF_planner.collision(agent)

        return False

    def agentCollision(self, otherAgent):

        return False
        
    def at_goal(self):

        if self.plannerType == 'APF':
            return self.APF_planner.at_goal()

        
        return True
