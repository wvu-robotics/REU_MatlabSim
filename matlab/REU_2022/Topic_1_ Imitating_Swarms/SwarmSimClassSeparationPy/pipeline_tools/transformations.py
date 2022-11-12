from scipy import signal
import numpy as np

from imitation_tools.data_prep import posVelSlice
# we could do it to positions too if that is better


def lowPassFilterPositions(agentPositions, cutoff):
    # need to make sure correct axis
    # data is by step, then by agent, then by component
    # by axis 1 should work
    sos = signal.butter(2, cutoff, output='sos')
    return signal.sosfilt(sos, agentPositions, axis=0)


# cutoff 0 to 1, closer to 0 is more filtering
# actually let me filter positions first, bc things get weird here with offsets
def lowPassFilterVels(pvsIn: list, cutoff):
    # need to reparse everything by agent vels
    # only smooth outputs, not agent observations
    pureVels = [pvs.next_vel for pvs in pvsIn]

    pureVels = np.array(pureVels)
    # print("Original",pureVels)

    # so it's now steps x agents x 2
    sos = signal.butter(2, cutoff, output='sos')

    # time axis is steps
    filteredVels = signal.sosfilt(sos, pureVels, axis=0).real

    # print("Filtered",filteredVels)
    filteredPVS = [posVelSlice(pvsIn[i].pos,pvsIn[i].vel,filteredVels[i]) for i in range(len(pvsIn))]
    return filteredPVS

def gaussianNoisePositions(agentPositions, mag, mu=0, sigma=1):
    return agentPositions + mag*np.random.normal(scale=sigma, size=agentPositions.shape)