import numpy as np
from sim_tools import sim
from tqdm import tqdm
from dataclasses import dataclass #data class is like the python equivalent of a struct
from sklearn.cluster import DBSCAN 
from models import Boids as bo

# class metrics_extract:

def get_metrics(gains,params=sim.SimParams):
    controllers = [bo.Boids(*gains) for i in range(params.num_agents)]
    print("First sim and export")
    agentPositions, agentVels = sim.runSim(controllers,params,progress_bar=True)
    iterations=5
    iter_values=[]
    Metrics=[]
    for j in range(iterations):

        @dataclass
        class posVelSlice:
            pos: np.ndarray
            vel: np.ndarray
            next_vel: np.ndarray


        posVelSlices = [posVelSlice(agentPositions[i],agentVels[i],agentVels[i+1]) for i in range(len(agentPositions)-1)]

    
            
        clusters=[]
        values= []
        
        distance_step=[]
        separation_step=[]
        cohesion_step=[]
        alignment_step=[]
        distance=[]
        separation=[]
        cohesion=[]
        alignment=[]
        average_distance=0
        coh=0
        alig=0
        sepr=0

        for step in tqdm(posVelSlices):
            distance=[]
            separation=[]
            cohesion=[]
            alignment=[]
            allPosStep=step.pos
            clustering = DBSCAN(eps=params.neighbor_radius, min_samples=2).fit(allPosStep)
            labels = clustering.labels_
            # print(labels)
            n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
            n_noise= list(labels).count(-1)
            # print("Estimated number of clusters: %d" % n_clusters)
            # print("Estimated number of noise points: %d" % n_noise)
            clusters.append(n_clusters)
            for agent in range(params.num_agents):
                
                posCentroid = np.zeros(2)
                velCentroid = np.zeros(2)
                sep = np.zeros(2)
                agentPos = step.pos[agent]
                agentVel = step.vel[agent]
                id_agent=labels[agent]
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
                    id_otherAgent=labels[otherAgent]
                    otherPos = step.pos[otherAgent]
                    otherVel = step.vel[otherAgent]
                    dist = np.linalg.norm(otherPos-agentPos)
                    
                    # if np.linalg.norm(otherPos-agentPos) > params.neighbor_radius:
                    #     continue
                    if id_agent == id_otherAgent and id_agent!= -1:                    
                        posCentroid += otherPos/dist
                        average_distance+=dist
                        sep += ((otherPos-agentPos)/dist)
                        # posCentroid += otherPos
                        velCentroid += otherVel
                        neighbors += 1
                    
                #throw out data without interactions
                if neighbors < 1:
                    continue
                separation.append(np.linalg.norm(sep))
                cohesion.append(np.linalg.norm(posCentroid))
                alignment.append(np.linalg.norm(velCentroid))
                average_distance/=neighbors
                distance.append(average_distance)
                values.append([cohesion,alignment,separation])

            distance_step.append(np.sum(distance)/len(distance))
            separation_step.append(np.sum(separation)/len(separation))
            cohesion_step.append(np.sum(cohesion)/len(cohesion))
            alignment_step.append(np.sum(alignment)/len(alignment))
        
        Metrics.append([np.mean(distance_step),
                np.mean(cohesion_step),
                np.mean(alignment_step),
                np.mean(separation_step)])
    
    METRICS=np.mean(Metrics)
    return METRICS
