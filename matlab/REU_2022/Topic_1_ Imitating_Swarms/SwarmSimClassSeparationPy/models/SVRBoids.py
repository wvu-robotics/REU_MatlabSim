from ctypes import alignment
import numpy as np
import GenericSwarmController
from dataclasses import dataclass

#inserting this dummy class here for debugging and printing purposes
@dataclass
class agentSlice:
    #inputs
    cohesion: np.ndarray
    alignment: np.ndarray
    separation: np.ndarray
    last_vel: np.ndarray
    #output
    output_vel: np.ndarray

#need to review paper, this is not working as expected, might need to add walls
class SVRBoids(GenericSwarmController.GenericSwarmController):
    def __init__(self, SVR_Model):
        self.SVR_Model = SVR_Model

    def vel(self,agentPositions,agentVels,pos,v):
        if(len(agentPositions) == 0):
            return v
        v_gain = np.zeros(2)

        centroidPos = np.zeros(2)
        for position in agentPositions:
            centroidPos += position
        centroidPos /= len(agentPositions)

        centroidVel = np.zeros(2)
        for vel in agentVels:
            centroidVel += vel
        centroidVel /= len(agentVels)


        #this is the force field approach, should eventually implement steer to avoid
        separation_out = np.zeros(2)
        for position in agentPositions:
            diffPos = position-pos
            dist = np.linalg.norm(diffPos)
            if(dist == 0):
                continue
            unit_diff = diffPos / dist
            if dist != 0:
                separation_out += -1*unit_diff*(1/(dist**6))

        x = [centroidPos-pos,centroidVel,separation_out]

        v_gain += self.SVR_Model.predict(x)

        v_out = v_gain
        # print(agentSlice(centroidPos-pos,centroidVel,separation_out,v,v_out))
        return v_out



