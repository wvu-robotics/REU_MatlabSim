import models.GenericSwarmController as GenericSwarmController
import numpy as np
import math
import random
from sim_tools.sim import SimParams
from shapely import geometry

#need to review paper, this is not working as expected, might need to add walls
class Boids(GenericSwarmController.GenericSwarmController):    
    def __init__(self,align_gain,cohesion_gain,separation_gain,inertia,params=SimParams()):
        self.alignment_gain = align_gain
        self.cohesion_gain = cohesion_gain
        self.separation_gain = separation_gain
        self.inertia = inertia
        self.params = params
        
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

        #steer-to-avoid
        origin = geometry.Point(pos[0],pos[1])

        # need to pass params for neighborhood stuff
        orientation = (v/np.linalg.norm(v))*self.params.neighbor_radius
        toward = geometry.Point(pos[0]+orientation[0],pos[1]+orientation[1])        

        velLine = geometry.LineString([origin,toward])

        separation = np.zeros(2)
        # this can be considered a property of the environment?, how is agent's size defined
        collision_distance = self.params.neighbor_radius/4

        closest = np.zeros(2)
        closestDist = np.inf

        for position in agentPositions:
            other = geometry.Point(position[0],position[1])
            collision = other.buffer(collision_distance)
            if velLine.intersects(collision):
                dist = np.linalg.norm(position-pos)
                if(dist < closestDist):
                    closestDist = dist
                    closest = position
        
        #assign separation
        if closestDist != np.inf:
            mag = 1/(closestDist**2)
            #figure out side
            diffPos = closest-pos
            #project onto v, grab remaining component as orthogonal direction
            v_hat = v/np.linalg.norm(v)
            d_v = np.dot(v_hat,diffPos)*v_hat
            remaining = diffPos - d_v
            remaining = remaining/np.linalg.norm(remaining)
            # print("Verify orthogonal: ",np.dot(remaining,v))
            # print("DiffPos",diffPos)
            # print("D_v",d_v)
            # print("D_r",remaining)
            # print("Sign",np.sign(np.cross(remaining,v)))
            separation = remaining*mag

        v_gain += self.separation_gain*separation

        v_out = v*self.inertia + v_gain
        return v_out