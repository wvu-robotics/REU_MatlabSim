import numpy as np
import sim_tools.sim as sim
from  sim_tools.sim import motionConstraints,neighborHood
import sim_tools.media_export as export
import copy
import os

import imitation_tools.automation as automation
from imitation_tools.data_prep import *
from models import linearSuperSet as lss
import plotly.express as px
import pandas as pd
from features.features import *
from models.featureCombo import FeatureCombo as fc
from scipy import optimize

from dataclasses import dataclass



params = sim.SimParams(
    num_agents=50,
    dt=0.05,
    overall_time = 15,
    enclosure_size = 15,
    init_pos_max = None, #if None, then defaults to enclosure_size
    agent_max_vel=7,
    init_vel_max = None,
    agent_max_accel=np.inf,
    agent_max_turn_rate=np.inf,
    neighbor_radius=3,
    periodic_boundary=False
    )



if __name__ == '__main__':
    orig_features = [
        Cohesion(),
        Alignment(), 
        SeparationInv2(),
        SteerToAvoid(params.neighbor_radius/4,params.neighbor_radius),
        Rotation()
    ]
    
    addGaussianNoise = False
   
    true_gains = np.array([1,1,1,1,1])

    orig = fc(true_gains,orig_features)
    controllers = [copy.deepcopy(orig) for i in range(params.num_agents)]

    #initial simulation
    print("First sim and export: ")
    agentPositions, agentVels = sim.runSim(controllers,params,progress_bar=True)

    if not os.path.exists("featureDecomp"):
        os.makedirs("featureDecomp")

    export.export(export.ExportType.GIF,"featureDecomp/Initial",agentPositions,agentVels,params=params,vision_mode=False,progress_bar=True)

    posVelSlices = toPosVelSlices(agentPositions,params)


    #  sigh we're rewriting this bc I keep changing by paradigms over and over again
    @dataclass
    class PVFeature:
        pvs:posVelSlice
        agentFeatureStrengths:np.ndarray #feature slice for each agent

    # parses features and multiplies them by weights, packs by agent for each pos vel slice
    PVFs =[PVFeature(pvs,np.array(
        [(np.array([np.zeros(2) if not len(n[0]) else f.compute(n[0],n[1],pvs.pos[i],pvs.vel[i])
            for n in [neighborHood(pvs.pos[i],params.neighbor_radius,pvs.pos,pvs.vel)]
                for f in orig_features]).transpose()*true_gains).transpose()
                    for i in range(len(pvs.pos))]))
                        for pvs in posVelSlices]

    print("Length of overall list",len(PVFs))
    print("length of each feature sublist",len(PVFs[0].agentFeatureStrengths))
    print("length of features",len(PVFs[0].agentFeatureStrengths[0]))


    names = ["Cohesion","Alignment","SeparationInv2","SteerToAvoid","Rotation"]
    # need to unwrap a lot of this to make it work
    # recompute sim by components
    # for j in range(len(names)):
    #     cohesionPositions = np.zeros([len(agentPositions),params.num_agents,2])
    #     cohesionPositions[0] = agentPositions[0]
    #     for i in range(1,len(PVFs)):
    #         currentPositions = cohesionPositions[i-1]
    #         # print(PVFs[i].agentFeatureStrengths.shape)
    #         # print(i)
    #         cohesionPositions[i] = currentPositions+ PVFs[i].agentFeatureStrengths[:,j]*params.dt
    #     export.export(export.ExportType.GIF,"featureDecomp/"+names[j],cohesionPositions,agentVels,params=params,vision_mode=False,progress_bar=True)
    
    
    # compare via subtraction
    posVelRotationResidual = [
        posVelSlice(pvf.pvs.pos,pvf.pvs.vel,
        pvf.pvs.next_vel - np.sum(pvf.agentFeatureStrengths[:,1:3],axis=1)) 
            for pvf in PVFs
    ]
    
    # # for i in range(len(PVFs)):
    #     print(i)
    #     print(PVFs[i].agentFeatureStrengths)

    # #shortsim params
    # shortSimParams = copy.deepcopy(params)
    # print("Running short sims")
    # shortSimParams.num_agents = 10
    # shortSimParams.enclosure_size = 10 #strong effect on learning separation
    # shortSimParams.overall_time = 4
    # shortSimParams.init_pos_max = shortSimParams.enclosure_size
    # shortSimParams.agent_max_vel = 7

    # learning_features = {
        
    #     "coh": Cohesion(),
    #     "align": Alignment(),
    #     "sep": SeparationInv2(),
    #     "sep6": SeparationInv6(),
    #     "steer": SteerToAvoid(params.neighbor_radius/4,params.neighbor_radius),
    #     "rot": Rotation()
    # }


    # featureSlices = automation.runSimsForFeatures(controllers,learning_features,num_sims=500,params=shortSimParams)