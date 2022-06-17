import models.GenericSwarmController as GenericSwarmController
import numpy as np
import math
import random
from shapely import geometry

#need to review paper, this is not working as expected, might need to add walls
class Boids(GenericSwarmController.GenericSwarmController):    
    def __init__(self,align_gain,cohesion_gain,inertia):
        self.alignment_gain = align_gain
        self.cohesion_gain = cohesion_gain
        self.inertia = inertia
        
    def vel(self,agentPositions,agentVels,pos,v):
        if(len(agentPositions) == 0):
            return v*self.inertia
        v_gain = np.zeros(2)
        
        centroidPos = np.zeros(2)
        for position in agentPositions:
            centroidPos += position
        centroidPos /= len(agentPositions)

        v_gain += self.cohesion_gain*(centroidPos-pos)

        #pretty sure I need some kind of 
        centroidVel = np.zeros(2)
        for vel in agentVels:
            centroidVel += vel
        centroidVel /= len(agentVels)

        v_gain += self.alignment_gain*centroidVel

        indicator1 = 0
        indicator2 = 0
        distance1 = [100, 100]
        distance2 = [100, 100]

        #steer-to-avoid
        for position in agentPositions:
            agent = position
            p0 = geometry.Point(agent[0],agent[1])
            vision = p0.buffer(1.0)
            velLine = geometry.LineString([agent,pos])
            left = velLine.parallel_offset(1, 'left')
            right = velLine.parallel_offset(1, 'right')
            endL = left.boundary[1]
            endR = right.boundary[0]
            #visionBound = geometry.LineString([endL, endR])
            slope = (((endR.y)-(endL.y))/((endR.x)-(endL.x)))
            intercept = -slope*(endL.x)+(endL.y)
            for position in agentPositions:
                p1 = geometry.Point(position[0],position[1])
                if vision.contains(p1):
                    if  position[1] > slope*position[0]+intercept:
                        if position[0] < agent[0]:
                            indicator1 += 1
                            distance = math.dist(position,v)
                            distance1.append(distance)
                        if position[0] > agent[0]:
                            indicator2 += 1
                            distance = math.dist(position,v)
                            distance2.append(distance)
            if indicator1 > indicator2:
                #turn towards triangle 2 (right)
                newX = np.cos(5.23599) - v*self.inertia * np.sin(5.23599)
                newY = np.sin(5.2359) + v*self.inertia * np.cos(5.2359)
                v_gain += 2*(newX + newY)
            if indicator2 > indicator1:
                #turn towards triangle 1 (left)
                newX = np.cos(1.0472) - v*self.inertia * np.sin(1.0472)
                newY = np.sin(1.0472) + v*self.inertia * np.cos(1.0472)
                v_gain += 2*(newX + newY)
            if indicator1 == indicator2:
                min1 = np.min(distance1)
                min2 = np.min(distance2)
                if min1 > min2:
                    #turn towards left
                    newX = np.cos(1.0472) - v*self.inertia * np.sin(1.0472)
                    newY = np.sin(1.0472) + v*self.inertia * np.cos(1.0472)
                    v_gain += 2*(newX + newY)
                if min2 > min1:
                    #turn towards right
                    newX = np.cos(5.23599) - v*self.inertia * np.sin(5.23599)
                    newY = np.sin(5.2359) + v*self.inertia * np.cos(5.2359)
                    v_gain += 2*(newX + newY)
                if min1 == min2 and min1 != 100:
                    choice = random.uniform(0,1)
                    if choice > 0.5:
                        newX = np.cos(5.23599) - v*self.inertia * np.sin(5.23599)
                        newY = np.sin(5.2359) + v*self.inertia * np.cos(5.2359)
                    else:
                        newX = np.cos(1.0472) - v*self.inertia * np.sin(1.0472)
                        newY = np.sin(1.0472) + v*self.inertia * np.cos(1.0472)
                    v_gain += 2*(newX+ newY)

            v_out = v*self.inertia + v_gain
        return v_out