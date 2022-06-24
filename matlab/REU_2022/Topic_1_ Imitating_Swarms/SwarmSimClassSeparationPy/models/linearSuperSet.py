import numpy as np
import models.GenericSwarmController as GenericSwarmController
from dataclasses import dataclass
from sim_tools.sim import SimParams
from shapely import geometry

#currently Boids + Dance

#need to review paper, this is not working as expected, might need to add walls
class SuperSet(GenericSwarmController.GenericSwarmController):    
    def __init__(self,cohesion_gain,align_gain,separation_gain,steer_to_avoid_gain,rotation_gain,inertia=1,params=SimParams()):
        self.alignment_gain = align_gain
        self.cohesion_gain = cohesion_gain
        self.separation_gain = separation_gain
        self.steer_to_avoid_gain = steer_to_avoid_gain
        self.rotation_gain = rotation_gain
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

        #agents move sideways relative to centroid
        relative_pos = np.zeros(2)
        relative_pos = centroidPos-pos
        k = relative_pos[1]
        relative_pos[1] = relative_pos[0]
        relative_pos[0] = -k
        v_gain += self.rotation_gain*(relative_pos/np.linalg.norm(relative_pos))

        v_gain += self.cohesion_gain*(centroidPos-pos)

        #pretty sure I need some kind of 
        centroidVel = np.zeros(2)
        for vel in agentVels:
            centroidVel += vel
        centroidVel /= len(agentVels)


        v_gain += self.alignment_gain*centroidVel

        #this is the force field approach, should eventually implement steer to avoid
        separation_out = np.zeros(2)
        for position in agentPositions:
            diffPos = position-pos
            dist = np.linalg.norm(diffPos)
            if(dist == 0):
                continue
            unit_diff = diffPos / dist
            if dist != 0:
                separation_out += -1*unit_diff*(1/(dist**2))
        
        v_gain += separation_out*self.separation_gain

        #steer-to-avoid
        origin = geometry.Point(pos[0],pos[1])

        # need to pass params for neighborhood stuff
        orientation = (v/np.linalg.norm(v))*self.params.neighbor_radius
        toward = geometry.Point(pos[0]+orientation[0],pos[1]+orientation[1])        

        velLine = geometry.LineString([origin,toward])

        steerToAvoid = np.zeros(2)
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
        
        #assign steerToAvoid
        if closestDist != np.inf:
            mag = 1/(closestDist**2)
            #figure out side
            diffPos = closest-pos
            #project onto v, grab remaining component as orthogonal direction
            v_hat = v/np.linalg.norm(v)
            d_v = np.dot(v_hat,diffPos)*v_hat
            remaining = diffPos - d_v
            if np.linalg.norm(remaining) > 0:
                remaining = remaining/np.linalg.norm(remaining)
            steerToAvoid = remaining*mag
        v_out = v_gain+steerToAvoid*self.steer_to_avoid_gain

        v_out = (v*self.inertia) + v_gain


        # print(agentSlice(centroidPos-pos,centroidVel,separation_out,v,v_out))
        return v_out



        