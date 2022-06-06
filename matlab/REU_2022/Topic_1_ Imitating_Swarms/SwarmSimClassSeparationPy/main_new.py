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
    init_pos_max= None, #if None, then defaults to enclosure_size
    agent_max_vel=5,
    agent_max_accel=np.inf,
    agent_max_turn_rate=np.inf,
    neighbor_radius=4,
    periodic_boundary=False
    )

#define list of controllers
controllers= [lj.LennardJones(1,100) for i in range(params.num_agents)]

agentPositions, agentVels = sim.runSim(controllers,params)
print("Sim finished -- Generating media")

#export types, NONE, INTERACTIVE, GIF, MP4
media_type = export.ExportType.INTERACTIVE

export.export(media_type,"new",agentPositions,params=params,vision_mode=False)
print("Media generated")