from bayes_opt import BayesianOptimization
import numpy as np
from sim_tools import sim
from tqdm import tqdm
from dataclasses import dataclass #data class is like the python equivalent of a struct
from models import Boids as bo
from macro_metrics_extract import get_metrics
from sim_tools import media_export as export

params = sim.SimParams(
    num_agents=30, 
    dt=0.05, 
    overall_time = 30, 
    enclosure_size = 10, 
    init_pos_max= None, #if None, then defaults to enclosure_size
    agent_max_vel=7,
    init_vel_max = None,
    agent_max_accel=np.inf,
    agent_max_turn_rate=np.inf,
    neighbor_radius=4.5,
    periodic_boundary=False
    )


#constants to imitate
k_coh = 8
k_align = 5
k_sep = 2
k_inertia = 1

true_gains = [k_coh, k_align, k_sep, k_inertia]

def black_box_function(r): #,coh, align, sep):
    
    # controllers = [bo.Boids(*true_gains) for i in range(params.num_agents)]
    # agentPositions, agentVels = sim.runSim(controllers,params,progress_bar=False)
    metrics=get_metrics(true_gains,params)

    params.neighbor_radius=r 
    # gains = [coh, align, sep, k_inertia]
    # controllersimitation = [bo.Boids(*true_gains) for i in range(params.num_agents)]
    # agentPositionsI, agentVelsI = sim.runSim(controllersimitation,params,progress_bar=False)
    metricsI=get_metrics(true_gains,params)

    # maxi=1/np.abs(np.array(metrics)-np.array(metricsI))
    max1=1/np.abs(metrics-metricsI)
    # max2=1/np.abs(metrics[1]-metricsI[2])

    
    return max1

pbounds = {'r': (1, 15)}#, 'coh': (0, 15), 'align': (0, 15), 'sep': (0, 15)}

optimizer = BayesianOptimization(
    f=black_box_function,
    pbounds=pbounds,
    random_state=1,
)

# optimizer.maximize(
#     init_points=2,
#     n_iter=6,
# )
optimizer.probe(
    params={"r": 4.5},
    lazy=True,
)
optimizer.maximize(init_points=0, n_iter=0)


optimizer.probe(
    params={"r": 4},
    lazy=True,
)
optimizer.maximize(init_points=0, n_iter=0)



# optimizer.probe(
#     params={"r": 10},
#     lazy=True,
# )
# optimizer.maximize(init_points=0, n_iter=0)