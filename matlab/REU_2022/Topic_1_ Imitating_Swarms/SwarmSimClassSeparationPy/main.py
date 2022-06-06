import numpy as np
import sim
import media_export as export

#model imports
from models import LenardJones as lj
from models import Boids as bo
from models import PFSM
from models import Dance

#all parameters for simulation
params = sim.SimParams(
    num_agents=100, 
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

#define list of controllers
controllers= [Dance.Dance(4,1,2,1) for i in range(params.num_agents)]

agentPositions, agentVels = sim.runSim(controllers,params)
print("Sim finished -- Generating media")

#export types, NONE, INTERACTIVE, GIF, MP4
media_type = export.ExportType.GIF

export.export(media_type,"new",agentPositions,params=params,vision_mode=True)
print("Media generated")