#even this will become another function eventually
from pipeline_tools.learning import learnMotionConstraints,learnNeighborRadius,learnGainsLinearRegression,learnGainsNonLinearOptimization
from pipeline_tools.prevalidation import testTrueNeighborRadius,testTrueGainsLinear,testTrueGainsNonLinear
from sim_tools import sim
from sim_tools import media_export as export
from imitation_tools import data_prep
import numpy as np

params = sim.SimParams(
    num_agents=30, 
    dt=0.05, 
    overall_time = 15, 
    enclosure_size = 15, 
    init_pos_max= None, #if None, then defaults to enclosure_size
    agent_max_vel=7,
    init_vel_max = None,
    agent_max_accel=np.inf,
    agent_max_turn_rate=np.inf,
    neighbor_radius=3,
    periodic_boundary=False
    )
