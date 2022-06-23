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
    return data_prep.toPosVelSlices(agentPositions,params),agentPositions,agentVels

# outputs posVel slices, multithreaded
# export info = [type,prefix name,every x iterations,colors] -- might generalize this elsewhere
def runSims(controllers=[],threads = 8,params=sim.SimParams(),num_sims=10,verbose=True,ignoreMC=False,ignoreBC=True,export_info =[]):
    if verbose:
        print("Running a batch of simulations")
    pool = Pool(threads)
    posVelClumps = list(tqdm(pool.imap(partial(runSimtoPosVel,params=params,controllers=controllers),range(num_sims)),total=num_sims))
    if export_info:
        if verbose: print("Exporting specific sims")
        for i in range(num_sims):
            if export_info and (i % export_info[2] == 0):
                if len(export_info) == 4:
                    for controller in controllers:
                        controller.setColor(export_info[3][int(i/export_info[2])]) #if you are doing rainbow, colors at each iteration
                if verbose:print("Sim "+str(i))
                export.export(export_info[0],export_info[1]+str(i),posVelClumps[i][1],posVelClumps[i][2],params=params,progress_bar=verbose)

    posVelSlices  = []
    for clump in posVelClumps:
        posVelSlices.extend(clump[0])
    agentSlices = data_prep.toAgentSlices(posVelSlices,params=params,verbose=verbose,
    ignoreConstrainedMotion=ignoreMC,ignoreBoundaryData=ignoreBC)
    return agentSlices

# non multithreaded version
# def runSims(controllers=[],params=sim.SimParams(),num_sims=10,verbose=True,export_info =[],ignoreMC=False,ignoreBC=True):
#     if verbose:
#         print("Running a batch of simulations")
#     posVelSlices = []
#     for shortSim in (tqdm(range(num_sims)) if verbose else range(num_sims)):
#         agentPositions,agentVels = sim.runSim(controllers,params)
#         if export_info and (shortSim % export_info[2] == 0):
#             if len(export_info) == 4:
#                 for controller in controllers:
#                     controller.setColor(export_info[3][int(shortSim/export_info[2])]) #if you are doing rainbow, colors at each iteration
#             export.export(export_info[0],export_info[1]+str(shortSim),agentPositions,agentVels,params=params)
#         posVelSlices.extend(data_prep.toPosVelSlices(agentPositions,params))
#     if verbose:
#         print("Parsing to agent slices")
#     agentSlices = data_prep.toAgentSlices(posVelSlices,params=params,verbose=verbose,
#     ignoreConstrainedMotion=ignoreMC,ignoreBoundaryData=ignoreBC)
#     return agentSlices

