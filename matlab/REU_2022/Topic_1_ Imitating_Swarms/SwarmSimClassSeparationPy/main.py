import numpy as np
import sim_tools.sim as sim
import sim_tools.media_export as export

#model imports
from models import LennardJones as lj
from models import Boids as bo
from models import PFSM
from models import Dance
from models import SteerControl as sc
from models import linearSuperSet as lss

#all parameters for simulation
params = sim.SimParams(
    num_agents=40, 
    dt=0.1, 
    overall_time = 30, 
    enclosure_size = 15, 
    init_pos_max= None, #if None, then defaults to enclosure_size
    agent_max_vel=5,
    init_vel_max = None,
    agent_max_accel=np.inf,
    agent_max_turn_rate=np.inf,
    neighbor_radius=3,
    periodic_boundary=False
    )

if __name__ ==  '__main__':
    #define list of controllers
    controllers= [sc.Boids(3,1,2,1,params=params) for i in range(params.num_agents)]

    #if you want to do coloring by agent
    for controller in controllers:
        # vals = np.random.uniform(0,1,3)*255
        # controller.setColor("rgb("+str(int(vals[0]))+","+str(int(vals[1]))+","+str(int(vals[2]))+")")
        controller.setColor("blue")


    agentPositions, agentVels = sim.runSim(controllers,params,progress_bar=True)
    print("Sim finished -- Generating media")

    #export types, NONE, INTERACTIVE, GIF, MP4
    media_type = export.ExportType.MP4

    export.export(media_type,"new",agentPositions,agentVels,controllers=controllers,params=params,vision_mode=False,progress_bar=True)
    print("Media generated")

    filename="sim_data"
    export.toCVS(filename,agentPositions,agentVels,params)