import numpy as np
import sim_mp as sim
import media_export as export
from timeit import default_timer as timer

#model imports
from models import LenardJones as lj
from models import Boids as bo
from models import PFSM
from models import Dance

#all parameters for simulation
params = sim.SimParams(
    num_agents=10, 
    dt=0.1, 
    overall_time = .3, 
    enclosure_size = 10, 
    init_pos_max= 3, #if None, then defaults to enclosure_size
    agent_max_vel=5,
    agent_max_accel=np.inf,
    agent_max_turn_rate=1.5*np.pi,
    neighbor_radius=3,
    periodic_boundary=False
    )

#define list of controllers
controllers= [bo.Boids(3,5,0.1,1) for i in range(params.num_agents)]

fullStart = timer()
agentPositions, agentVels = sim.runSim(controllers,params,progress_bar=False)
simDone = timer()
print("Sim finished -- Generating media")

#export types, NONE, INTERACTIVE, GIF, MP4
media_type = export.ExportType.GIF

exportStart = timer()
export.export(media_type,"bench",agentPositions,agentVels,params=params,vision_mode=False,progress_bar=True)
exportDone = timer()

print("Media generated")

print("Full time: ", exportDone - fullStart)
print("Sim time: ", simDone - fullStart)
print("Export time: ", exportDone - exportStart)
