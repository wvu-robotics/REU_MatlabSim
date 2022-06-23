import numpy as np
import sim_tools.sim as sim
import sim_tools.media_export as export
import time
import copy
import cmath

from sklearn.linear_model import LinearRegression as lr
from tqdm import tqdm
from sklearn.metrics import mean_absolute_error
from sklearn.svm import SVR
from sklearn.model_selection import train_test_split
from sklearn.multioutput import MultiOutputRegressor
from sklearn.gaussian_process import GaussianProcessRegressor as gp
from dataclasses import dataclass #data class is like the python equivalent of a struct
from models import Boids as bo
from models import BoidsNonLinear as nl
from msvr.model.MSVR import MSVR #library written by someone else

#I might switch back to xgb, but trying  with this for now, to do multioutput
from sklearn.ensemble import AdaBoostRegressor
from sklearn.tree import DecisionTreeRegressor

#I want to try decision trees with boosting and multi output and see if that works

import sys
sys.setrecursionlimit(2147483647)

params = sim.SimParams(
    num_agents=40,
    dt=0.05,
    overall_time = 15,
    enclosure_size = 10,
    init_pos_max= None, #if None, then defaults to enclosure_size
    agent_max_vel=5,
    init_vel_max = 2,
    agent_max_accel=np.inf,
    agent_max_turn_rate=np.inf,
    neighbor_radius=3,
    periodic_boundary=False
    )

if __name__ ==  '__main__':
    #constants to imitate
    k_coh = 3
    k_align = 5
    k_sep = 1
    k_inertia = 1

    true_gains = [k_coh, k_align, k_sep, k_inertia]

    #run sim
    print("Generating Initial")
    controllers = [bo.Boids(*true_gains) for i in range(params.num_agents)]
    agentPositions, agentVels = sim.runSim(controllers,params,progress_bar=True)

    export.export(export.ExportType.GIF,"SVRInitial",agentPositions,agentVels,params=params,vision_mode=False,progress_bar=True)

    # print("Agent positions:")
    # print(agentPositions)
    # print("Agent velocities:")
    # print(agentVels)

    #reformat positions and vels by slice
    @dataclass
    class posVelSlice:
        pos: np.ndarray
        vel: np.ndarray
        next_vel: np.ndarray

    posVelSlices = [posVelSlice(agentPositions[i],agentVels[i],agentVels[i+1]) for i in range(len(agentPositions)-1)]

    # print("PosVelSlices:")
    # print(posVelSlices)

    posVelSlices = []
    #run tons more short sims
    shortSimParams = copy.deepcopy(params)
    print("Running short sims")
    shortSimParams.overall_time = 2
    # shortSimParams.enclosure_size = 2*params.enclosure_size
    # shortSimParams.init_pos_max = params.enclosure_size/2

    extra_sims = 100
    for extra_sim in tqdm(range(extra_sims)):
        agentPositions, agentVels = sim.runSim(controllers,shortSimParams)
        posVelSlices.extend([posVelSlice(agentPositions[i],agentVels[i],agentVels[i+1]) for i in range(len(agentPositions)-1)])


    #reformat slices to be relevant by agent, getting the centroid, separation, alignment, and last vel
    @dataclass
    class agentSlice:
        #inputs
        cohesion: np.ndarray
        alignment: np.ndarray
        separation: np.ndarray
        last_vel: np.ndarray
        #output
        output_vel: np.ndarray

    agentSlices = []
    for slice in posVelSlices:
        for agent in range(params.num_agents):
            #calculate all relevant derivate metrics, repackage
            posCentroid = np.zeros(2)
            velCentroid = np.zeros(2)
            agentPos = slice.pos[agent]
        
            #throw out data near the boundary, only needed with bounce
            if(agentPos[0] > params.enclosure_size or agentPos[0] < -params.enclosure_size 
            or agentPos[1] > params.enclosure_size or agentPos[1] < -params.enclosure_size):
                continue

            agentVel = slice.vel[agent]
            agentNextVel = slice.next_vel[agent]
            separation = np.zeros(2)

            adjacent = 0
            for otherAgent in range(params.num_agents):
                if otherAgent == agent:
                    continue
                
                otherPos = slice.pos[otherAgent]

                if np.linalg.norm(otherPos-agentPos) > params.neighbor_radius:
                    continue
                
                dist = np.linalg.norm(otherPos-agentPos)

                separation += ((otherPos-agentPos)/dist)*-1*(1/(dist**6))
                otherVel = slice.vel[otherAgent]
                posCentroid += otherPos
                velCentroid += otherVel
                adjacent += 1
            
            #throw out data without interactions
            if adjacent < 1:
                continue

            posCentroid /= adjacent
            velCentroid /= adjacent

            agentVelDifference = agentNextVel - agentVel

            agentSlices.append(agentSlice(posCentroid-agentPos,velCentroid,separation,np.zeros(2),agentVelDifference))

    #sanity check, this should have 0 loss, anything else is either noise or bad measurements
    true_loss = 0
    for slice in agentSlices:
        gains_ex = true_gains[:3]
        args = np.array([slice.cohesion,slice.alignment,slice.separation])
        # print("Args",args)
        # print("Gains",gains_ex)
        vel_pred = np.dot(args.transpose(),gains_ex)
        # print("Pred",vel_pred)
        # need to add scaling
        
    #angle deviation of velocity
        if(np.linalg.norm(vel_pred) > 0 and np.linalg.norm(agentVel) > 0):
            vel_pred_angle = np.arctan2(vel_pred[1],vel_pred[0])
            vel_angle = np.arctan2(agentVel[1],agentVel[0])
            angleDeviation = vel_pred_angle - vel_angle
            if angleDeviation > np.pi or angleDeviation < -np.pi:
                angleDeviation = -2*np.pi + angleDeviation

            maxAngleDeviation = params.agent_max_accel*params.dt

            if abs(angleDeviation) > maxAngleDeviation:
                agent_vel_hat = agentVel/np.linalg.norm(agentVel)
                agent_vel_complex = agent_vel_hat[0] + 1j*agent_vel_hat[1]
                agent_vel_complex *= np.linalg.norm(vel_pred)
                # figure out which side to rotate
                if vel_angle + maxAngleDeviation > np.pi:
                    maxAngleDeviation += 2*np.pi
                elif vel_angle - maxAngleDeviation < -1*np.pi:
                    maxAngleDeviation -= 2*np.pi

                if angleDeviation > 0:
                    agent_vel_complex *= cmath.exp(1j*-1*maxAngleDeviation)
                elif angleDeviation < 0:
                    agent_vel_complex *= cmath.exp(1j*maxAngleDeviation)
                else:
                    pass
                vel_pred = np.array([agent_vel_complex.real,agent_vel_complex.imag])

        if(np.linalg.norm(vel_pred) > params.agent_max_vel):
            vel_pred = vel_pred/np.linalg.norm(vel_pred)*params.agent_max_vel

        maxVelChange = params.agent_max_accel*params.dt

        if np.linalg.norm(vel_pred)-np.linalg.norm(agentVel) > maxVelChange:
            if np.linalg.norm(vel_pred) == 0:
                vel_pred = agentVel + maxVelChange*(agentVel/np.linalg.norm(agentVel))
            else:
                vel_pred = vel_pred*((maxVelChange+np.linalg.norm(agentVel))/np.linalg.norm(vel_pred)) 
        elif np.linalg.norm(vel_pred)-np.linalg.norm(agentVel) < -maxVelChange:
            if np.linalg.norm(vel_pred) == 0:
                vel_pred = agentVel - maxVelChange*(agentVel/np.linalg.norm(agentVel))
            else:
                vel_pred = vel_pred*((-maxVelChange+np.linalg.norm(agentVel))/np.linalg.norm(vel_pred))

        # print("Pred scaled",vel_pred)
        # print("output",slice.output_vel)

        err = vel_pred - slice.output_vel
        #print("err",err)
        true_loss += np.linalg.norm(err)
    true_loss/=len(agentSlices)
    print("True loss(data deviation from original on average):",true_loss)


    print("Ignored ",(len(posVelSlices)*params.num_agents)-len(agentSlices),"/",len(posVelSlices)*params.num_agents," slices")

    x = []
    y = []

    np.random.shuffle(agentSlices) # because we are only sampling some of the data
    for slice in agentSlices:
        x.append(np.array([slice.cohesion[0],slice.alignment[0],slice.separation[0],slice.cohesion[1],slice.alignment[1],slice.separation[1]]))
        y.append(slice.output_vel)

    x = np.array(x)
    y = np.array(y)

    # print(x)
    # print(y.shape)


    #clipping a lot of data here, might do smart subsampling
    #train 2 SVRs for the purpose, one for x, one for y

    #no need for double training ever, use sklearn's multioutput regressor
    x_train, x_test, y_train, y_test = train_test_split(x, y, random_state=1, test_size=0.3)


    # svr_rbf = SVR(kernel='rbf', gamma=0.1)
    # svr_linear = SVR(kernel='linear')
    # svr_poly0 = SVR(kernel='poly',gamma='auto',degree=3,tol=0.1)
    # svr_poly1 = SVR(kernel='poly',gamma='auto',degree=3,tol=0.1)

    # svr_sig0 = SVR(kernel='sigmoid')
    # svr_sig1 = SVR(kernel='sigmoid')

    # svr_rbf0 = SVR(kernel='rbf')
    # svr_rbf1 = SVR(kernel='rbf')

    msvr_model = MSVR(kernel='sigmoid', epsilon=0.005)

    #can do ridge regression or GPR or anything else here, but I wanna try trees
    multi_model = MultiOutputRegressor(gp(copy_X_train=False))
    #AdaBoostRegressor(DecisionTreeRegressor(max_depth=7),n_estimators=1000)


    # xgb_model = xgb.XGBRegressor(learning_rate=0.0001, n_estimators=1000)

    def evaluate_model(name,model, x_train, y_train, x_test, y_test,num_batches=1):
        print(name)
        start = time.time()
        if num_batches == 1:
            model.fit(x_train, y_train)
        else:
            batch_size  = int(len(x_train)/num_batches)
            for batch in range(num_batches):
                i_start = batch*batch_size
                i_end = min((batch+1)*batch_size,len(x_train))
                model.fit(x_train[i_start:i_end], y_train[i_start:i_end])
        y_pred = model.predict(x_test)
        ypred_train = model.predict(x_train)
        print('MSE train', mean_absolute_error(y_train, ypred_train))
        print('MSE test', mean_absolute_error(y_test, y_pred))
        r_2 = model.score(x_test, y_test)
        print('R^2 test', r_2)
        print('Execution time: {0:.2f} seconds.'.format(time.time() - start))
        return r_2
        # print(name + '($R^2={:.3f}$)'.format(r_2), np.array(y_test), y_pred)


    # evaluate_model("XGB",xgb_model,x_train,y_train,x_test,y_test)
    # evaluate_model("SVR_poly0",svr_poly0,x_train0,y_train0,x_test0,y_test0)
    # evaluate_model("SVR_poly1",svr_poly1,x_train1,y_train1,x_test1,y_test1)
    # evaluate_model("SVR_sig0",svr_sig0,x_train0,y_train0,x_test0,y_test0)
    # evaluate_model("SVR_sig1",svr_sig1,x_train1,y_train1,x_test1,y_test1)
    # evaluate_model("SVR_rbf0",svr_rbf0,x_train0,y_train0,x_test0,y_test0)
    # evaluate_model("SVR_rbf1",svr_rbf1,x_train1,y_train1,x_test1,y_test1)

    # evaluate_model("MSVR",msvr_model,x_train,y_train,x_test,y_test)

    batch_size = 100
    num_batches = int(len(x_train)/batch_size)
    evaluate_model("Multi",multi_model,x_train,y_train,x_test,y_test,num_batches=num_batches)


    # evaluate_model("SVR_rbf",svr_rbf,x_train,y_train,x_test,y_test)
    # evaluate_model("SVR_linear",svr_linear,x_train,y_train,x_test,y_test)
    # evaluate_model("SVR_sigmoid",svr_sig,x_train,y_train,x_test,y_test)

    print("Exporting")
    controllers_imitated = [nl.Boids(multi_model,inertia=k_inertia) for i in range(params.num_agents)]
    for controller in controllers_imitated:
        controller.setColor('black')
    agentPositions_imitated, agentVels_imitated = sim.runSim(controllers_imitated,params,progress_bar=True)
    export.export(export.ExportType.GIF,"SVRImitated",agentPositions_imitated,agentVels_imitated,controllers=controllers_imitated,params=params,progress_bar=True)
