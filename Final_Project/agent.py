import math
import matplotlib.pyplot as plt
from APF import *
from ExpansivePlanner import *

# Constants

class Agent():

    def __init__(self, isPursuer, pose, goal_pose, plannerType):

        # Save room for both planners, later specify which one is not None
        self.APF_planner = None
        self.exp_planner = None

        # Keep track of it
        self.plannerType = plannerType

        # Boolean to tell whether or not we're the pursuer
        self.isPursuer = isPursuer

        # Keep track of pose, if pursuer, ignore goal pose
        self.pose = pose
        self.goal_pose = goal_pose

        # Planner Type
        self.setPlanner(plannerType)
    
    def setPlanner(plannerType):

        if plannerType == 'APF':
            self.APF_planner = APF()

        else:
            self.exp_planner = ExpansivePlanner()

    def getNextPose(time_stamp):

        if plannerType == 'APF':
            return self.APF_planner.getNextPose(time_stamp)

        else:
            return self.exp_planner.getNextPose(time_stamp)

    def collision(self):

        return False

    def agentCollision(self, otherAgent):

        return False
        
    def at_goal(self):
        
        return True
