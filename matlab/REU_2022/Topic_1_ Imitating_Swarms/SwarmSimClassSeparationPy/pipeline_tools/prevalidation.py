from pickletools import optimize
from imitation_tools.data_prep import posVelSlice, featureSlice, toFeatureSlices
from pipeline_tools.learning import fitnessLinearReg, dataForLinReg, sliceBasedLoss
from sim_tools.sim import SimParams
from functools import partial
import plotly.express as px
import numpy as np

# tests to ensure data is well formed(ASSUMING FULL PRIOR KNOWLEDGE, DEBUGGING)

# use exact fitness function


def testTrueNeighborRadius(posVelSlices: list[posVelSlice], orig_params: SimParams, learning_features: dict):
    fitness_fn = fitnessLinearReg(posVelSlices, orig_params, learning_features)
    return -1*fitness_fn([orig_params.neighbor_radius], 0)  # returning loss


def plotNeighborRadius(posVelSlices: list[posVelSlice], orig_params: SimParams, learning_features: dict):
    fitness_fn = fitnessLinearReg(posVelSlices, orig_params, learning_features)
    x = np.linspace(0, 10, 50)
    y = [fitness_fn([a],0) for a in x]
    fig = px.line(x=x, y=y)
    fig.update_layout(title="Neighbor Radius")
    fig.show()


def testTrueGainsLinear(true_gains, featureSlices: list[featureSlice], params: SimParams):
    # validating true gains in same way as lin reg(optimally should )
    x_flat, y_flat=dataForLinReg(featureSlices, params, verbose = False)
    true_loss=0
    for i in range(len(x_flat)):
        v_pred=np.dot(x_flat[i].transpose(), true_gains)
        v_actual=y_flat[i]
        true_loss += np.sum(np.abs(v_pred-v_actual))
    return true_loss

def testTrueGainsNonLinear(true_gains, featureSlices: list[featureSlice], params: SimParams, maxSample: int = 5000):
    np.random.shuffle(featureSlices)  # since we subsample the data
    loss_fn=sliceBasedLoss(
        featureSlices[:min(len(featureSlices), maxSample)], params)
    return loss_fn(true_gains)
