import numpy as np
import sim_tools.sim as sim
import sim_tools.media_export as export
from tqdm import tqdm

from functools import partial
from multiprocessing import Pool

from imitation_tools import data_prep

#dummy var needed to use map--just using apply might be cleaner form, but this is cleaner code
def runSimtoPosVel(dummy,controllers=[],params=sim.SimParams()):
    agentPositions,agentVels = sim.runSim(controllers,params)
    return data_prep.toPosVelSlices(agentPositions,params)

# no export so far
def runSimsMP(controllers=[],threads = 8,params=sim.SimParams(),num_sims=10,verbose=True,ignoreMC=False,ignoreBC=True):
    if verbose:
        print("Running a batch of simulations")
    pool = Pool(threads)
    posVelClumps = list(tqdm(pool.imap(partial(runSimtoPosVel,params=params,controllers=controllers),range(num_sims)),total=num_sims))
    posVelSlices  = []
    for clump in posVelClumps:
        posVelSlices.extend(clump)
    agentSlices = data_prep.toAgentSlices(posVelSlices,params=params,verbose=verbose,
    ignoreConstrainedMotion=ignoreMC,ignoreBoundaryData=ignoreBC)
    return agentSlices

# outputs posVel slices
# export info = [type,prefix name,every x iterations,colors] -- might generalize this elsewhere
def runSims(controllers=[],params=sim.SimParams(),num_sims=10,verbose=True,export_info =[],ignoreMC=False,ignoreBC=True):
    if verbose:
        print("Running a batch of simulations")
    posVelSlices = []
    for shortSim in (tqdm(range(num_sims)) if verbose else range(num_sims)):
        agentPositions,agentVels = sim.runSim(controllers,params)
        if export_info and (shortSim % export_info[2] == 0):
            if len(export_info) == 4:
                for controller in controllers:
                    controller.setColor(export_info[3][int(shortSim/export_info[2])]) #if you are doing rainbow, colors at each iteration
            export.export(export_info[0],export_info[1]+str(shortSim),agentPositions,agentVels,params=params)
        posVelSlices.extend(data_prep.toPosVelSlices(agentPositions,params))
    if verbose:
        print("Parsing to agent slices")
    agentSlices = data_prep.toAgentSlices(posVelSlices,params=params,verbose=verbose,
    ignoreConstrainedMotion=ignoreMC,ignoreBoundaryData=ignoreBC)
    return agentSlices

