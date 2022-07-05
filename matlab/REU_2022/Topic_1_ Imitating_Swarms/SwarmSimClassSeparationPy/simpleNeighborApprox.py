from imitation_tools.data_prep import posVelSlice
from sim_tools import sim
from sim_tools import media_export as export
from imitation_tools import data_prep

from models.Boids import Boids
import numpy as np
import copy
import os

params = sim.SimParams(
    num_agents=40, 
    dt=0.01, 
    overall_time = 10, 
    enclosure_size = 10, 
    init_pos_max= None, #if None, then defaults to enclosure_size
    agent_max_vel=9.23,
    init_vel_max = None,
    agent_max_accel=1,
    agent_max_turn_rate=10*np.pi,
    neighbor_radius=2,
    periodic_boundary=False
    )


if __name__ ==  '__main__':
    k_coh = 3
    k_align = 1
    k_sep = .2

    orig_controller = Boids(k_coh,k_align,k_sep,1)
    # really should clean up this interface too
    controllers = [copy.deepcopy(orig_controller) for i in range(params.num_agents)]
    agentPositions, agentVels = sim.runSim(controllers,params,progress_bar=True)

    if not os.path.exists("SimpleNeighbor"):
        os.makedirs("SimpleNeighbor")

    export.export(export.ExportType.GIF,"SimpleNeighbor/Initial",agentPositions,agentVels,params=params,vision_mode=False,progress_bar=True)


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
                
                #currently moves by dt..., .1 dt will give original, need to figure out why
                if np.linalg.norm(back2Vel)>maxVel:
                    maxVel = np.linalg.norm(back2Vel)

                #note currently done by magnitude, should really do it in 2D the correct way
                velChange = np.linalg.norm(previousVel)-np.linalg.norm(back2Vel)
                accel = velChange
                if accel>maxAccel:
                    maxAccel = accel
                
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
                        # print("Last Deviation",np.linalg.norm(previousVel-back2Vel))
                        # print("Recent Deviation",np.linalg.norm(currentVel-previousVel))
                        # print("new neighbor radius: ", neighborRadius)



    print("Estimated neighbor radius: ", neighborRadius) #good
    print("Estimated Max Velocity: ", maxVel) # good
    print("Estimated Max Acceleration: ", maxAccel) # still some big problems