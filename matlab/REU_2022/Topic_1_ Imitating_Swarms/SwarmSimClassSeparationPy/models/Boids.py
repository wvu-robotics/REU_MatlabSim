import numpy as np
import models.GenericSwarmController as GenericSwarmController


#need to review paper, this is not working as expected, might need to add walls
class Boids(GenericSwarmController.GenericSwarmController):    
    def __init__(self,align_gain,cohesion_gain,separation_gain,inertia):
        self.alignment_gain = align_gain
        self.cohesion_gain = cohesion_gain
        self.separation_gain = separation_gain
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

        separation_out = np.zeros(2)
        for position in agentPositions:
            diffPos = position-pos
            dist = np.linalg.norm(diffPos)
            if(dist == 0):
                continue
            unit_diff = diffPos / dist
            if dist != 0:
                separation_out += -1*self.separation_gain*unit_diff
        
        v_gain += separation_out
        
        v_out = (v*self.inertia) + v_gain
        return v_out



        