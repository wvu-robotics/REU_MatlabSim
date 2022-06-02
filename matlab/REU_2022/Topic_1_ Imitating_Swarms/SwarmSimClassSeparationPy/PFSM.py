import GenericSwarmController
import pydtmc
from pydtmc import MarkovChain 
import numpy as np
import plotly.express as px
import pandas as pd
from enum import Enum

class States(Enum):
    Forward = 0
    Backward = 1
    Freeze = 2
    Flee = 3

class PFSM(GenericSwarmController.GenericSwarmController):
    def __init__(self):
        self.state = States.Forward.name
        self.p = [[0.30, 0.30, 0.40, 0.00], [0.35, 0.25, 0.20, 0.20], [0.20, 0.30, 0.30, 0.20], [0.30, 0.20, 0.40, 0.10]]
        self.mc = MarkovChain(self.p, [States.Forward.name, States.Backward.name, States.Freeze.name, States.Flee.name])
    
    def vel(self, agentPositions, agentVels, pos, v):
        centroidPos = np.zeros(2)
        for position in agentPositions:
            if len(agentPositions) == 0:
                centroidPos = np.zeros(2)
            else:
                centroidPos /= len(agentPositions)

        v_gain = (centroidPos-pos)


        self.state = pydtmc.MarkovChain.next_state(self=self.mc, initial_state=self.state) 
        v_out = np.zeros(2)
        if self.state == States.Forward.name:
            v_out = v_gain+v_gain*20                
        elif self.state == States.Backward.name:
            v_out = v_gain+v_gain*-20
        elif self.state == States.Freeze.name:
            v_out = np.zeros(2)
        elif self.state == States.Flee.name:
            v_out = v_gain+v_gain*100
        return v_out