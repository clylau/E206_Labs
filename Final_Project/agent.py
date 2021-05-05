import math
import matplotlib.pyplot as plt
from utilities import *
from APF import *
from Expansive_Planner import * #Dear Prof Clark, Christina is literally the worst partner I've ever had the displeasure of working with. Will you please just fail me. Love, Jimmy
#from ExpansivePlanner import *

# Constants
goal_threshold = 0.5 #m

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

        else:
            if self.id == 0:
                goal_weight = 0.8
                opponent_weight = 0.2
                v_min = 1.5
                v_max = 1.75
            else:
                goal_weight = 0.7
                opponent_weight = 0.3
                v_min = 1.75
                v_max = 2

            self.exp_planner = Expansive_Planner(goal_weight, opponent_weight, v_min, v_max)

    def updateGoalPose(self, new_goal):
        self.goal_pose = new_goal

    def update(self, time_stamp, delta_t, agent_list, obj_list, world_edge):
        # if self.id == 0:
        #     print('Pose of Evader: ', self.pose.x, self.pose.y, self.pose.theta)
        # if self.id == 1:
        #     print('Goal of Pursuer: ', self.goal_pose.x, self.goal_pose.y, self.goal_pose.theta)

        if self.plannerType == 'APF':
            return self.APF_planner.update(delta_t, agent_list, obj_list, world_edge)

        else:
            if(time_stamp - self.exp_planner.last_update > self.exp_planner.update_rate):
                self.exp_planner.update_traj(self.pose, self.goal_pose, time_stamp, obj_list, world_edge, self.id, agent_list)


            return self.exp_planner.update(time_stamp + delta_t, self)

            #TODO: update the trajectory

        # if self.id == 1:
        #     self.updateGoalPose(agent_list[0].pose)
        return None


    def collision(self, agent):
        """ Function to check if the agent is in collision with another agent.
            Arguments:
            agent (APF_agent): The other agent there could be a collision with.
            Returns:
            collision (Bool): True if there is a collision.
        """
        dist = math.sqrt((self.pose.x - agent.pose.x)**2 + (self.pose.y - agent.pose.y)**2)
        return dist < self.radius + agent.radius #and self.alive and agent.alive

    def out_of_bounds(self, world_length):
        """ Function to check if the agent is out of bounds.
            Arguments:
            world_radius (float): The radius of the world in m.
            Returns:
            out_of_bounds (Bool): True if the agent is out of bounds.
        """
        if np.abs(self.pose.x) >= world_length-self.radius or np.abs(self.pose.y) >= world_length-self.radius:
            return True

        #dist = math.sqrt(self.pose.x**2 + self.pose.y**2)
        #return dist > world_radius - self.radius

    # def collision(self, agent):
    #     # if self.plannerType == 'APF':
    #     #     return self.APF_planner.collision(agent)

    #     return False

    def agentCollision(self, otherAgent):

        return False
        
    def at_goal(self, agentList):
        
        #if the robot is the evader, it needs to reach its goal (w/in epislon)
        if(self.id == 0):

            dist = np.sqrt(np.square(self.pose.x - self.goal_pose.x) + np.square(self.pose.y - self.goal_pose.y))
            at_goal = dist < goal_threshold

        #if the robot is the pursuer, it needs to collide with its goal (the other robot - so epsilon is that robots radius)
        else:

            #assuming that the agent list is in order of evader, pursuer
            evader_radius = agentList[0].radius
            dist = np.sqrt(np.square(self.pose.x - self.goal_pose.x) + np.square(self.pose.y - self.goal_pose.y))
            at_goal = dist <= evader_radius + self.radius

        
        return at_goal
