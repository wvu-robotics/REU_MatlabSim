import numpy as np
from sim_tools import sim
from tqdm import tqdm
from dataclasses import dataclass #data class is like the python equivalent of a struct


# class metrics_extract:

def get_metrics(agentPositions,agentVels,params=sim.SimParams):

    @dataclass
    class posVelSlice:
        pos: np.ndarray
        vel: np.ndarray
        next_vel: np.ndarray


    posVelSlices = [posVelSlice(agentPositions[i],agentVels[i],agentVels[i+1]) for i in range(len(agentPositions)-1)]

  
        

    values= []
    separation=0
    cohesion=0
    alignment=0
    coh=0
    alig=0
    sepr=0

    for slice in tqdm(posVelSlices):

        for agent in range(params.num_agents):
            
            posCentroid = np.zeros(2)
            velCentroid = np.zeros(2)
            sep = np.zeros(2)
            agentPos = slice.pos[agent]
            agentVel = slice.vel[agent]
            #throw out data near the boundary, only needed with bounce
            if(agentPos[0] > params.enclosure_size or agentPos[0] < -params.enclosure_size 
            or agentPos[1] > params.enclosure_size or agentPos[1] < -params.enclosure_size):
                continue
                    
            

            # throw out data that is at the edge of action constraints--might add some probability
            # if np.linalg.norm(agentVel) >= params.agent_max_vel:
            #     continue
            neighbors= 0
            for otherAgent in range(params.num_agents):
                if otherAgent == agent:
                    continue
                
                otherPos = slice.pos[otherAgent]
                otherVel = slice.vel[otherAgent]

                if np.linalg.norm(otherPos-agentPos) > params.neighbor_radius:
                    continue
                
                dist = np.linalg.norm(otherPos-agentPos)
                sep += ((otherPos-agentPos)/dist)
                
                posCentroid += otherPos
                velCentroid += otherVel
                neighbors += 1
            
            #throw out data without interactions
            if neighbors < 1:
                continue
            # sep/= neighbors
            # # print(neighbors)
            # posCentroid /= neighbors
            # velCentroid /= neighbors
            separation= np.linalg.norm(sep)
            cohesion = np.linalg.norm(posCentroid)
            alignment = np.linalg.norm(velCentroid)

            values.append([cohesion,alignment,separation])

    values=np.array(values)
    # print(values)
    metrics=[]
    for i in range(len(values)):
        coh=values[i][0]+coh
        alig=values[i][1]+alig
        sepr=values[i][2]+sepr
    coh/=len(values)
    alig/=len(values)
    sepr/=len(values)
    metrics=[coh,alig,sepr]
    

    return metrics
