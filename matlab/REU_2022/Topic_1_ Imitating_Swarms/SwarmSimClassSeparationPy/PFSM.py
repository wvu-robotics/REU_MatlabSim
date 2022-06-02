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
        self.state = States.Forward
    
    def vel(self, agentPositions, agentVels, pos, v):
        centroidPos = np.zeros(2)
        for position in agentPositions:
            centroidPos += position
        centroidPos /= len(agentPositions)

        v_gain = (centroidPos-pos)
        if self.state == States.Forward:
            v_out = v_gain*3                
        elif self.state == States.Backward:
             v_out = v_gain*-3
        elif self.state == States.Freeze:
             v_out = v_gain
        elif self.state == States.Flee:
             v_out = v_gain+2
        return v_out