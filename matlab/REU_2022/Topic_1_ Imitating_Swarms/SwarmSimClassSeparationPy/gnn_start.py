# first just import swarm data to a graph, then go from there

# hmm what I even want in this graph
# Node contents
# current pos
# current vel
# last vel?
# a list of neighbors stuff? or is that implicit

# data will be a node features matrix X
# and a node adjacency matrix A
# hrm but I need to do this for every timestep
# network X is a good library for this

from imitation_tools.data_prep import posVelSlice
from sim_tools import sim
from sim_tools import media_export as export
from imitation_tools import data_prep

from models.featureCombo import FeatureCombo as fc
import numpy as np
import copy
import os

from features.features import *


params = sim.SimParams(
    num_agents=40, 
    dt=0.01, 
    overall_time = 10, 
    enclosure_size = 10, 
    init_pos_max= None, #if None, then defaults to enclosure_size
    agent_max_vel=5,
    init_vel_max = None,
    agent_max_accel=np.inf,
    agent_max_turn_rate=np.inf,
    neighbor_radius=3,
    periodic_boundary=False
    )


if __name__ ==  '__main__':
    orig_features = [
        Cohesion(),
        Alignment(),
        SeparationInv2()
    ]

    orig_controller = fc([1,1,1],orig_features)
    # really should clean up this interface too
    controllers = [copy.deepcopy(orig_controller) for i in range(params.num_agents)]
    agentPositions, agentVels = sim.runSim(controllers,params,progress_bar=True)

    if not os.path.exists("SimpleNeighbor"):
        os.makedirs("SimpleNeighbor")

    # export.export(export.ExportType.GIF,"SimpleNeighbor/Initial",agentPositions,agentVels,params=params,vision_mode=False,progress_bar=True)


    shortSimParams = copy.deepcopy(params)
    print("Running short sim")
    shortSimParams.overall_time = 1
    num_sims = 4

    posVelSlices = []
    neighborRadius = 0
    maxVel = 0
    maxAccel = 0
    # posVelSlices = data_prep.toPosVelSlices(agentPositions)
    for i in range(num_sims):
        trainPositions, trainVels = sim.runSim(controllers,shortSimParams,progress_bar=True)
        posVelSlices = data_prep.toPosVelSlices(trainPositions,params=shortSimParams)
    
        for i in range(2,len(posVelSlices)):
            current = posVelSlices[i] #not using posvel slices next vel for clarity
            previous = posVelSlices[i-1]
            back2 = posVelSlices[i-2]


            # check agent by agent
            for agent in range(params.num_agents):
                currentVel = current.vel[agent]
                previousVel = previous.vel[agent]
                back2Vel = back2.vel[agent]
                

                
                currentPos = current.pos[agent]
                previousPos = previous.pos[agent]
                back2Pos = back2.pos[agent]

                # make sure to ignore boundary conditions(probably ignore full posvel slice now)
                boundaryBroken = False
                for pos in [currentPos,previousPos,back2Pos]:
                    if pos[0]<-params.enclosure_size or pos[0]>params.enclosure_size or pos[1]<-params.enclosure_size or pos[1]>params.enclosure_size:
                        boundaryBroken = True
                        break
                if boundaryBroken: continue

