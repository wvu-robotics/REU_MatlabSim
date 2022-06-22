import numpy as np
import sim_tools.sim as sim
import sim_tools.media_export as export
from tqdm import tqdm

from imitation_tools import data_prep


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

