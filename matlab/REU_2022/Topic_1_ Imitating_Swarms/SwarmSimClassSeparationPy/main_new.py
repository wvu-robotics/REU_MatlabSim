import numpy as np
import sim
import media_export as export

#model imports
import LenardJones as lj
import Boids as bo
import PFSM

#all parameters for simulation
params = sim.SimParams(
    num_agents=100, 
    dt=0.1, 
    overall_time = 15, 
    enclosure_size = 25, 
    init_pos_max= 3, #if None, then defaults to enclosure_size
    agent_max_vel=5,
    agent_max_accel=1.5*np.pi,
    agent_max_turn_rate=np.inf,
    neighbor_radius=4,
    periodic_boundary=False
    )

#define list of controllers
controllers= [bo.Boids(1,3,0.4,1) for i in range(params.num_agents)]

#ERROR: there is an issue with this code sending them all to 0, need to fix
agentPositions, agentVels = sim.runSim(controllers,params)

#export types, NONE, INTERACTIVE, GIF, MP4
media_type = export.ExportType.GIF

export.export(media_type,"new",agentPositions,params=params,vision_mode=False)