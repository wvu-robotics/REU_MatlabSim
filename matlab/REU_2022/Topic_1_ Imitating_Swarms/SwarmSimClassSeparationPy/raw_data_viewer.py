from sim_tools import sim
from sim_tools import media_export as export
from imitation_tools import data_prep
from features.features import *
from pipeline_tools import transformations
from models.featureCombo import FeatureCombo as fc

from tqdm import tqdm
import numpy as np
import copy
import os

if __name__ == '__main__':
    dir = 'teaching_swarm_data/Rezec_robots/'
    filename = 'Rezec_Robot_Swarm_Data.npz'
    datazip = np.load(dir+filename)

    num_agents = datazip['number_agents']
    overall_time = datazip['overall_time']
    steps = datazip['steps']

    agentPositions = datazip['agent_positions']

    # enclosure bounding might need to be adjusted for non square
    enclosure_edge = datazip['enclosure_size']

    print("Imported data in:")
    print("Num agents", num_agents)
    print("Overall time", overall_time)
    print("Steps", steps)
    print("Enclosure Size", enclosure_edge)
    print("Agent Positions", agentPositions.shape)

    overall_time = overall_time  # just speed it up the dumb way, not smoothing

    # steps technically refers to transformations, so 1 less than list of steps
    steps = steps-0.5
    learnedParams = sim.SimParams()
    learnedParams.overall_time = overall_time
    learnedParams.dt = overall_time/steps
    learnedParams.num_agents = num_agents
    learnedParams.enclosure_size = enclosure_edge

    if not os.path.exists("Output/Raw_Data_Viewer"):
        os.makedirs("Output/Raw_Data_Viewer")

    print("Exporting raw data to visualization")
    export.export(export.ExportType.GIF, "Output/Raw_Data_Viewer/"+filename+"view",
                  agentPositions, np.zeros(agentPositions.shape), learnedParams, progress_bar=True, framecap=15)  # velocities not used in export

    agentPositions = transformations.lowPassFilterPositions(
        agentPositions, 0.2)

    export.export(export.ExportType.GIF, "Output/Raw_Data_Viewer/"+filename+"viewFiltered",
        agentPositions, np.zeros(agentPositions.shape), learnedParams, progress_bar=True, framecap=15)  # velocities not used in export
