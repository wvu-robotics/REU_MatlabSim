import models.GenericSwarmController as GenericSwarmController
import numpy as np

class Dance(GenericSwarmController.GenericSwarmController):

    def __init__(self,rotation_gain,cohesion_gain, wiggle_gain, inertia):
        self.rotation_gain = rotation_gain
        self.cohesion_gain = cohesion_gain
        self.wiggle_gain = wiggle_gain
        self.inertia = inertia


    def vel(self, agentPositions, agentVels, pos, v):
            if (len(agentPositions) == 0):
                return v * self.inertia
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

            #agents are cohesive
            v_gain += self.cohesion_gain * (centroidPos - pos)

            #agents wiggle
            wiggle_vector = np.zeros(2)
            wiggle_vector[0] = np.random.uniform(-1,1)
            wiggle_vector[1] = np.random.uniform(-1,1)
            wiggle_vector /= np.linalg.norm(wiggle_vector)
            v_gain += self.wiggle_gain * wiggle_vector

            v_out = self.inertia*v + v_gain
            return v_out




