import GenericSwarmController
import pydtmc
from pydtmc import MarkovChain 
import numpy as np
import plotly.express as px
import pandas as pd
from enum import Enum
from ctypes import alignment

class States(Enum):
    Forward = 0
    Backward = 1
    Freeze = 2
    Flee = 3

class PFSM(GenericSwarmController.GenericSwarmController):
    def __init__(self,align_gain,cohesion_gain,separation_gain,inertia):
        self.state = States.Forward.name
        self.alignment_gain = align_gain
        self.cohesion_gain = cohesion_gain
        self.separation_gain = separation_gain
        self.inertia = inertia
        self.p = [[0.30, 0.30, 0.40, 0.00], [0.35, 0.25, 0.20, 0.20], [0.20, 0.30, 0.30, 0.20], [0.30, 0.20, 0.40, 0.10]]
        self.mc = MarkovChain(self.p, [States.Forward.name, States.Backward.name, States.Freeze.name, States.Flee.name])
    
    def vel(self, agentPositions, agentVels, pos, v):
        centroidPos = np.zeros(2)
        v_gain = np.zeros(2)
        for position in agentPositions:
            if len(agentPositions) == 0:
                centroidPos = np.zeros(2)
            else:
                centroidPos /= len(agentPositions)
        v_gain += self.cohesion_gain*(centroidPos-pos)

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
        #v_gain = (centroidPos-pos)
 
        self.state = pydtmc.MarkovChain.next_state(self=self.mc, initial_state=self.state)
        v_out = np.zeros(2)

        if self.state == States.Forward.name:
            v_out = v_gain+v_gain*200
        elif self.state == States.Backward.name:
            v_out = v_gain+v_gain*-200
        elif self.state == States.Freeze.name:
            v_out = np.zeros(2)
        elif self.state == States.Flee.name:
            v_out = v_gain+v_gain*2000
        return v_out