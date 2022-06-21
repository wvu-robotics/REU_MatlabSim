#thoughts - get rid of inertia, have them follow last heading if no interaction
import numpy as np
import sim_tools.sim as sim
import sim_tools.media_export as export
from tqdm import tqdm

from dataclasses import dataclass

@dataclass
class posVelSlice:
    pos: np.ndarray
    vel: np.ndarray
    next_vel: np.ndarray

def toPosVelSlices(agentPositions,params=sim.SimParams()):
    #includes velocity calculation--I don't fully understand how to abstract this out yet
    # would really like to remove this dt
    return [posVelSlice(agentPositions[i],
        (agentPositions[i]-agentPositions[i-1])/params.dt,
        (agentPositions[i+1]-agentPositions[i])/params.dt)
    for i in range(1,len(agentPositions)-1)]

#will need to be expanded for each nonlinear feature(there might be a nice way to generalize)
@dataclass
class agentSlice:
    #inputs
    cohesion: np.ndarray
    alignment: np.ndarray
    separation: np.ndarray
    rotation: np.ndarray
    last_vel: np.ndarray
    #output
    output_vel: np.ndarray

def toAgentSlices(posVelSlices,params=sim.SimParams(),neighborCaps=[1,np.inf],ignoreConstrainedMotion=False,ignoreBoundaryData=True,verbose=True):
    agentSlices = []
    for slice in (tqdm(posVelSlices) if verbose else posVelSlices):
        for agent in range(params.num_agents):
            #calculate all relevant derivate metrics, repackage
            posCentroid = np.zeros(2)
            velCentroid = np.zeros(2)
            agentPos = slice.pos[agent]

            agentVel = slice.vel[agent]
            agentNextVel = slice.next_vel[agent]
            separation = np.zeros(2)

            if ignoreBoundaryData:
                #throw out data near the boundary, only needed with bounce
                if(agentPos[0] > params.enclosure_size or agentPos[0] < -params.enclosure_size 
                or agentPos[1] > params.enclosure_size or agentPos[1] < -params.enclosure_size):
                    continue

                if np.linalg.norm(agentNextVel) >= params.agent_max_accel:
                  continue
        
                vel_new_angle = np.arctan2(agentNextVel[1],agentNextVel[0])
                vel_angle = np.arctan2(agentVel[1],agentVel[0])
                angleDeviation = vel_new_angle - vel_angle
                if angleDeviation > np.pi or angleDeviation < -np.pi:
                    angleDeviation = -2*np.pi + angleDeviation
        
                if abs(angleDeviation) >= params.agent_max_turn_rate*params.dt:
                    continue
            
            if ignoreConstrainedMotion:
                if np.linalg.norm(agentVel) >= params.agent_max_vel:
                    continue

            adjacent = 0
            for otherAgent in range(params.num_agents):
                if otherAgent == agent:
                    continue
                
                otherPos = slice.pos[otherAgent]

                if np.linalg.norm(otherPos-agentPos) > params.neighbor_radius:
                    continue
                
                dist = np.linalg.norm(otherPos-agentPos)

                separation += ((otherPos-agentPos)/dist)*-1*(1/(dist**6))
                otherVel = slice.vel[otherAgent]
                posCentroid += otherPos
                velCentroid += otherVel
                adjacent += 1
            
            #agents move sideways relative to centroid
            relative_pos = np.zeros(2)
            relative_pos = posCentroid-agentPos
            k = relative_pos[1]
            relative_pos[1] = relative_pos[0]
            relative_pos[0] = -k
            rotation = relative_pos/np.linalg.norm(relative_pos)

            #throw out data without interactions
            if adjacent < neighborCaps[0] or adjacent > neighborCaps[1]:
                continue

            posCentroid /= adjacent
            velCentroid /= adjacent

            agentSlices.append(agentSlice(posCentroid-agentPos,velCentroid,separation,rotation,agentVel,agentNextVel))
        # not sure why this is broken
    if verbose:
        print("Ignored ",(len(posVelSlices)*params.num_agents)-len(agentSlices),"/",len(posVelSlices)*params.num_agents," slices")
    return agentSlices

