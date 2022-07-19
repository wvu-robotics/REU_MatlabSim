from  models.GenericSwarmController import GenericSwarmController
import numpy as np

class FeatureComboEnv(GenericSwarmController):
    # features and gains need to be the same length
    def __init__(self,social_gains,social_features,env_gains,environmental_features,inertia=1):
        self.social_features = social_features
        self.environmental_features = environmental_features
        self.social_gains = social_gains
        self.env_gains = env_gains
        self.inertia = inertia
    def vel(self,agentPositions,agentVels,pos,v):
        v_gain = np.zeros(2)
        if(len(agentPositions) == 0):
            for i in range(len(self.environmental_features)):
                environmental_component = self.env_gains[i]*self.environmental_features[i].compute(agentPositions,agentVels,pos,v)
                v_gain += environmental_component
            return v_gain + v*self.inertia
        for i in range(len(self.environmental_features)):
            environmental_component = self.env_gains[i]*self.environmental_features[i].compute(agentPositions,agentVels,pos,v)
            v_gain += environmental_component
        for i in range(len(self.social_features)):
            # yeah I think shape is causing problems, need to debug functions
            social_component = self.social_gains[i]*self.social_features[i].compute(agentPositions,agentVels,pos,v)
            # print("Shape",component.shape)
            # print("Component",component)
            # print("i",i)
            v_gain += social_component
        return v_gain + (v*self.inertia)#, getting rid of inertia worked ALOT