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
    noise_percent = 0.7
    for pos in agentPositions:
        # 0.6 to account for stdev value, so most things are within bounds
        # maybe should do the noise based on current vel
        noise = 0.6*noise_percent*params.agent_max_vel*np.random.normal(0,1,size=(2))*params.dt
        pos+=noise

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

                # make sure to ignore boundary conditions
                boundaryBroken = False
                for pos in [currentPos,previousPos,back2Pos]:
                    if pos[0]<-params.enclosure_size or pos[0]>params.enclosure_size or pos[1]<-params.enclosure_size or pos[1]>params.enclosure_size:
                        boundaryBroken = True
                        break
                if boundaryBroken: continue

                #currently moves by dt..., .1 dt will give original, need to figure out why
                if np.linalg.norm(back2Vel)>maxVel:
                    maxVel = np.linalg.norm(back2Vel)

                #note currently done by magnitude, should really do it in 2D the correct way
                velChange = abs(np.linalg.norm(previousVel)-np.linalg.norm(back2Vel))
                accel = velChange
                if accel>maxAccel:
                    maxAccel = accel

                # was moving in straight line before, now changed
                # maybe need to add a larger tolerance
                if (np.isclose(previousVel, back2Vel)).all() and not (np.isclose(previousVel,currentVel)).all():
                    minDist = np.inf
                    for otherAgent in range(params.num_agents):
                        if otherAgent != agent:
                            dist = np.linalg.norm(current.pos[agent]-current.pos[otherAgent])
                            if dist < minDist:
                                minDist = dist
                    if minDist != np.inf and minDist > neighborRadius:
                        neighborRadius = minDist



    print("Estimated neighbor radius: ", neighborRadius) #good
    print("Estimated Max Velocity: ", maxVel) # good
    print("Estimated Max Acceleration: ", maxAccel) # still some big problems