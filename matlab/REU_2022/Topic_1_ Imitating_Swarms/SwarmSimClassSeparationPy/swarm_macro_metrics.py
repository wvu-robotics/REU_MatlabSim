from random import random
import numpy as np
import sim
import media_export as export
import matplotlib.pyplot as plt
from tqdm import tqdm
from dataclasses import dataclass #data class is like the python equivalent of a struct
from models import Boids as bo
iterations=8
iter_values=[]
params = sim.SimParams(
    num_agents=5, 
    dt=0.05, 
    overall_time = 15, 
    enclosure_size = 10, 
    init_pos_max= None, #if None, then defaults to enclosure_size
    agent_max_vel=7,
    init_vel_max = None,
    agent_max_accel=np.inf,
    agent_max_turn_rate=np.inf,
    neighbor_radius=3,
    periodic_boundary=False
    )



#constants to imitate
k_coh = 3
k_align = 5
k_sep = 1
k_inertia = 1

true_gains = [k_coh, k_align, k_sep, k_inertia]

#run sim
# print("Original agent slices")
for j in range(iterations):

    controllers = [bo.Boids(*true_gains) for i in range(params.num_agents)]
    print("First sim and export")
    agentPositions, agentVels = sim.runSim(controllers,params,progress_bar=True)
    # export.export(export.ExportType.GIF,"Initial",agentPositions,params=params,vision_mode=False,progress_bar=True)

    @dataclass
    class posVelSlice:
        pos: np.ndarray
        vel: np.ndarray
        next_vel: np.ndarray


    posVelSlices = [posVelSlice(agentPositions[i],agentVels[i],agentVels[i+1]) for i in range(len(agentPositions)-1)]

    @dataclass
    class stepSlice:
        #inputs
        cohesion: np.ndarray
        alignment: np.ndarray
        separation: np.ndarray
        

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
            # print(agentPos)
        
            #throw out data near the boundary, only needed with bounce
            # if(agentPos[0] > params.enclosure_size or agentPos[0] < -params.enclosure_size 
            # or agentPos[1] > params.enclosure_size or agentPos[1] < -params.enclosure_size):
            #     continue
                    
            

            # # throw out data that is at the edge of action constraints--might add some probability
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
            sep/= neighbors
            # print(neighbors)
            posCentroid /= neighbors
            velCentroid /= neighbors
            separation= np.linalg.norm(sep)
            cohesion = np.linalg.norm(posCentroid)
            alignment = np.linalg.norm(velCentroid)

            values.append([cohesion,alignment,separation])
    # print(values)
    values=np.array(values)
    
    for i in range(len(values)):
        coh=values[i][0]+coh
        alig=values[i][1]+alig
        sepr=values[i][2]+sepr
    coh/=len(values)
    alig/=len(values)
    sepr/=len(values)
    np.array(iter_values.append([coh,alig,sepr]))
print("________")
print(iter_values)

print(len(iter_values))
C=[]
A=[]
S=[]
for index, value in enumerate(iter_values):
    C.append(value[0])
    A.append(value[1])
    S.append(value[2])

index=[*range(1,iterations+1)]

fig, axs = plt.subplots(3,sharex=True)
fig.suptitle('macro metrics')
plt.ylim(0,10)
axs[0].plot(index,C,'o')
axs[1].plot(index,A,'o')
axs[2].plot(index,S,'o')
axs[0].set(ylabel='cohesion')
axs[0].set_ylim([0,10])
axs[1].set(ylabel='alignment')
axs[1].set_ylim([0,10])
axs[2].set(ylabel='separation')
axs[2].set_ylim([0,10])

plt.xlabel('iteration number')
plt.show(fig)
# values=[]
# for step in stepSlices:
#     values.append(np.array([step.posCentroid,step.velCentroid,step.separation ]))
    
# values=np.vstack(values)
# print(len(values[:,0]))