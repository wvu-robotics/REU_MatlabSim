from abc import abstractmethod
import numpy as np

#eventually want to export neighborRadius into main.py
class GenericSwarmController:
    @abstractmethod
    def vel(self,agentPositions,agentVels,pos,v):
        pass

    def setColor(self,color):
        self.color = color
