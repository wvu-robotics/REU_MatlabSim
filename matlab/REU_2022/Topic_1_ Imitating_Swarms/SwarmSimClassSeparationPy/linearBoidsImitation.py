import numpy as np
import sim
import media_export as export
from dataclasses import dataclass #data class is like the python equivalent of a struct

from models import Boids as bo

params = sim.SimParams(
    num_agents=10, 
    dt=0.1, 
    overall_time = 15, 
    enclosure_size = 25, 
    init_pos_max= None, #if None, then defaults to enclosure_size
    agent_max_vel=5,
    agent_max_accel=1,
    agent_max_turn_rate=1.5*np.pi,
    neighbor_radius=5,
    periodic_boundary=False
    )

#constants to imitate
k_coh = 1
k_align = 3
k_sep =0.4
k_inertia = 1

#run sim
controllers = [bo.Boid(k_coh,k_align,k_sep,k_inertia) for i in range(params.num_agents)]
agentPositions, agentVels = sim.runSim(controllers,params)

#reformat positions and vels by slice
@dataclass
class posVelSlice:
    pos: np.ndarray
    vel: np.ndarray
    next_vel: np.ndarray

posVelSlices = [posVelSlice(agentPositions[i],agentVels(i),agentVels[i+1]) for i in range(len(agentPositions)-1)]

#reformat slices to be relevant by agent, getting the centroid, separation, alignment, and last vel
@dataclass
class agentSlice:
    centroid: np.ndarray
    separation: np.ndarray
    alignment: np.ndarray
    last_vel: np.ndarray

agentSlices = []
for slice in posVelSlices:
    for agent in range(params.num_agents):
        #calculate all relevant derivate metrics, repackage
        pass

#do linear regression on these things, and get the coefficients, do some train/test split

#create some visuals with the imitated swarm
    #maybe do an imposter(s) pretending to be within the original swarm