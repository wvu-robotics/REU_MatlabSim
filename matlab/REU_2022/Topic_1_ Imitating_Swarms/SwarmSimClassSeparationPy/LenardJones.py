import numpy as np
import GenericSwarmController

class LennardJones(GenericSwarmController.GenericSwarmController):
    def __init__(self,sigma,epsilon):
        self.sigma = sigma
        self.epsilon = epsilon


    def vel(self,agentPositions,agentVels,pos,v):
        v_gain = np.zeros(2)

        #find distance vectors for all nearby
        for position in agentPositions:
            diffPos = position-pos
            dist = np.linalg.norm(diffPos)
            unit_diff = diffPos / dist

            #non linearities

            #lennard-jones, actually works
            epsilon = self.epsilon
            sigma = self.sigma
            out = epsilon*(((sigma/dist)**12)+(-2*((sigma/dist)**6)))

            if out != 0:
                v_gain += out*unit_diff
        inertia = 1
        v_out = (v*inertia) + v_gain
        return v_out
