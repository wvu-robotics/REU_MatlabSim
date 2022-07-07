import numpy as np
import sim_tools.sim as sim
import sim_tools.media_export as export
from timeit import default_timer as timer

#model imports
from models import LennardJones as lj
from models import Boids as bo
from models import PFSM
from models import Dance
from models import linearSuperSet as lss

#all parameters for simulation
params = sim.SimParams(
    num_agents=40, 
    dt=0.01, 
    overall_time = 30, 
    enclosure_size = 15, 
    init_pos_max= None, #if None, then defaults to enclosure_size
    agent_max_vel=9,
    init_vel_max = None,
    agent_max_accel=np.inf,
    agent_max_turn_rate=np.inf,
    neighbor_radius=3,
    periodic_boundary=False
    )
if __name__ ==  '__main__':
    #define list of controllers
    controllers= [lss.SuperSet(1,1,1,1,1,params=params) for i in range(params.num_agents)]

    fullStart = timer()
    agentPositions, agentVels = sim.runSim(controllers,params,progress_bar=True)
    simDone = timer()
    print("Sim finished -- Generating media")

    #export types, NONE, INTERACTIVE, GIF, MP4
    media_type = export.ExportType.MP4

    exportStart = timer()
    export.export(media_type,"bench",agentPositions,agentVels,params=params,vision_mode=False,progress_bar=True)
    exportDone = timer()

    print("Media generated")

    print("Full time: ", exportDone - fullStart)
    print("Sim time: ", simDone - fullStart)
    print("Export time: ", exportDone - exportStart)
