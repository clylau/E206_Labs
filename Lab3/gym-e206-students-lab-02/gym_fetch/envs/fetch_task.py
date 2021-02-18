import gym
import gym_fetch
import time
import math
import random

class PathTracker():

    current_point_to_track = 0
    path = []
    MIN_DIST_TO_POINT = 0.25
    MIN_ANG_TO_POINT = 0.10
    path_tracked = False
    
    def __init__(self, path):
        self.current_point_to_track = 0
        self.path = path
        self.path_tracked = False

    def get_point_to_track(self, x):
        
        dist_to_current_point = math.sqrt(pow(x[0]-self.path[self.current_point_to_track][0],2)+pow(x[1]-self.path[self.current_point_to_track][1],2))
        if dist_to_current_point < self.MIN_DIST_TO_POINT and abs(x[2]-self.path[self.current_point_to_track][2])<self.MIN_ANG_TO_POINT:
            if self.current_point_to_track == len(self.path)-1:
                self.path_tracked = True
                print('Path Tracked: ',len(self.path))
            self.current_point_to_track = min(len(self.path)-1,self.current_point_to_track+1)
            print('Now Tracking: ',self.current_point_to_track,' ',self.path[self.current_point_to_track])
        
        x_to_track = self.path[self.current_point_to_track]
        return x_to_track
    
    def print_path(self):
        print("Path:")
        for i in range(len(self.path)):
            print(i,self.path[i])
            
    def is_path_tracked(self):
        return self.path_tracked
    
class PointTracker():

    def __init__(self):
        pass

    def get_action(self, x_des, x):
        action = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
        return action

    def angle_diff(self, ang):
        while ang > math.pi:
            ang -= 2*math.pi 
            
        while ang < -math.pi:
            ang += 2*math.pi
            
        return ang

    def point_tracking_control(self, x_des, x):

        # zero all of action
        action = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
    
        # Some useful variables
        K_rho = 1*1.0
        K_alpha = 1*2.0
        K_beta = -0.99
        robotRadius = 0.2
        robotWidth = robotRadius*2
        wheelRadius = 0.1
        
        # Set thresholds
        posTrackingThreshold = 0.1
        angTrackingThreshold = 0.1
        trackingThresholds = [posTrackingThreshold, angTrackingThreshold]

        # Some helpful calculations
        deltaX = x_des[0] - x[0]
        deltaY = x_des[1] - x[1]
        ang = math.atan2(deltaY,deltaX)


        # Calucate rho, the distance to goal
        rho = math.sqrt( deltaX*deltaX + deltaY*deltaY )

        # Check to see if simply rotation on the spot, if rho is small
        if (rho < trackingThresholds[0]):
            rho = 0
            alpha = 0
            beta = self.angle_diff(x[2]- x_des[2])

        # if rho is still too big, invoke the point tracking theory
        else:
            alpha = self.angle_diff(-x[2] + ang)
            if abs(alpha) < math.pi/2:
                beta = self.angle_diff(self.angle_diff(-x[2] - alpha) + x_des[2])
            else:
                ang = math.atan2(-deltaY,-deltaX)
                alpha = self.angle_diff(-x[2] + ang)
                beta = self.angle_diff(self.angle_diff(-x[2] - alpha) + x_des[2])
                rho = - rho

        # Calculate v, w
        v = K_rho * rho
        w = K_alpha * alpha + K_beta * beta

        # calculate in m/s, convert to rad/s
        desiredWheelSpeedR = (v + w*robotWidth)*(2*math.pi)/(2*math.pi*wheelRadius)
        desiredWheelSpeedL = (v - w*robotWidth)*(2*math.pi)/(2*math.pi*wheelRadius) #was -ve

        # Check that max velocity in rad/s isn't reached
        currentMaxVel = max(abs(desiredWheelSpeedL), abs(desiredWheelSpeedR))
        maxWheelSpeed = 2.0*math.pi;
        if  currentMaxVel > maxWheelSpeed:
            desiredWheelSpeedR = desiredWheelSpeedR * maxWheelSpeed / currentMaxVel
            desiredWheelSpeedL = desiredWheelSpeedL * maxWheelSpeed / currentMaxVel
        

        # Set the control vector U
        action[0] = desiredWheelSpeedR
        action[1] = desiredWheelSpeedL

        return action
    
    

def main():
    env = gym.make("fetch-v0") # <-- this we need to create
    env.render('human')
    env.reset()
    
    controller = PointTracker()
    observation = [0,0,0,0,0]
    x_des = [3,0,0]
        
    time.sleep(5)
    for _ in range(1):
        
        path = [[0,1,0,0]]
        path_tracker = PathTracker(path)
        path_tracker.print_path()
        while path_tracker.is_path_tracked() == False:

            point_to_track = path_tracker.get_point_to_track([observation[0], observation[1], observation[2]])
            action = controller.point_tracking_control(point_to_track, observation)
            observation, reward, done, dummy = env.step(action) 
            env.render('human')
            
    time.sleep(2)
    env.close()

if __name__ == '__main__':
    main()
    
    
