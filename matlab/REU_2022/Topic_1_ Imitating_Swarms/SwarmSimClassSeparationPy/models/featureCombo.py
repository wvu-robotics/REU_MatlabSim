from  models.GenericSwarmController import GenericSwarmController
import numpy as np

class FeatureCombo(GenericSwarmController):
    # features and gains need to be the same length
    def __init__(self,gains,features,inertia=1):
        self.features = features
        self.gains = gains
        self.inertia = inertia
    def vel(self,agentPositions,agentVels,pos,v):
        if(len(agentPositions) == 0):
            return v*self.inertia
        v_gain = np.zeros(2)
        for i in range(len(self.features)):
            # yeah I think shape is causing problems, need to debug functions
            component = self.gains[i]*self.features[i].compute(agentPositions,agentVels,pos,v)
            # print("Shape",component.shape)
            # print("Component",component)
            # print("i",i)
            v_gain += component
        return v_gain + (v*self.inertia)#, getting rid of inertia worked ALOT