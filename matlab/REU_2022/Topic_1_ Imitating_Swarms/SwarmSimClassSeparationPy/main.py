import numpy as np
import plotly.express as px
import plotly.graph_objects as go
import pandas as pd
import cmath
from PIL import Image
import cv2
import os


#model imports
import LenardJones as lj
import Boids as bo

# for video encoding you need FFMPEG installed for opencv, or VFW on windows

#toggles whether output to interactive graph or a video/gif
interactive_graph = False

# sim parameters
overallTime = 15 # seconds
dt = .1
steps = int(overallTime/dt)
numAgents = 10
neighborRadius = 4

# BCs
isPeriodic = True

#agent motion constraints
angularRate = 1.5*np.pi #radians per second
maxAngleDeviation = angularRate*dt #radians per step
agentMaxVel = 5 # m/s
agentMaxAccel = 1 # m/s^2
agentMaxVelChange = agentMaxAccel*dt #m/s per step

# define arrays of positions and vels over each step
agentPositions = np.zeros([steps+1,numAgents,2])
agentVels = np.zeros([steps+1,numAgents,2])

#base inertia stuff broken, need to fix
agentControllers = [bo.Boids(1,3,1,1) for i in range(numAgents)]

# initial positions and velocities
enclosureSize = 25
randPosMax = 4

for agentPos in agentPositions[0]:
    #probably a way to assign whole array at once, but this is more explicit
    agentPos[0] = np.random.uniform(low=-randPosMax,high=randPosMax)
    agentPos[1] = np.random.uniform(low=-randPosMax,high=randPosMax)

# print(agentPositions[0])
# print(agentPositions[0,0])

for agentVel in agentVels[0]:
    agentVel[0] = np.random.uniform(-agentMaxVel,agentMaxVel)  
    agentVel[1] = np.random.uniform(-agentMaxVel,agentMaxVel)

#run simulation
for step in range(0,steps):
   for agent in range(0,numAgents):
        agentPos = agentPositions[step,agent]
        agentVel = agentVels[step,agent]

        relevantPositions = []
        relevantVels = []

        #don't like passing global info
        for other_agent in range(0,numAgents):
            if agent == other_agent or np.linalg.norm(agentPos-agentPositions[step,other_agent]) > neighborRadius:
                continue
            relevantPositions.append(agentPositions[step,other_agent])
            relevantVels.append(agentVels[step,other_agent])

        #if out of bounds, invert last velocity and don't update
        if isPeriodic == True:
            pass
        else:
            if agentPos[0] > enclosureSize or agentPos[0] < -enclosureSize:
                if((agentVel[0]<0 and agentPos[0]<-enclosureSize) or (agentVel[0]>0 and agentPos[0]>enclosureSize)):
                    agentVel[0] *= -1
                agentVels[step+1,agent] = agentVel*(agentMaxVel/np.linalg.norm(agentVel))
                agentPositions[step+1,agent] = agentPos  + agentVel*dt
                continue

            if agentPos[1] > enclosureSize or agentPos[1] < -enclosureSize:
                if((agentVel[1]<0 and agentPos[1]<-enclosureSize) or (agentVel[1]>0 and agentPos[1]>enclosureSize)):
                    agentVel[1] *= -1
                agentVels[step+1,agent] = agentVel*(agentMaxVel/np.linalg.norm(agentVel))
                agentPositions[step+1,agent] = agentPos  + agentVel*dt
                continue

        #periodic BCs - neighbor radius does not cross periodic boundaries
        if isPeriodic == True:
            if agentPos[0] > enclosureSize or agentPos[0] < -enclosureSize:
                if agentVel[0]<0 and agentPos[0]<-enclosureSize:
                    agentPos[0] += 2*enclosureSize
                elif agentVel[0]>0 and agentPos[0]>enclosureSize:
                    agentPos[0] += -2*enclosureSize

            if agentPos[1] > enclosureSize or agentPos[1] < -enclosureSize:
                if agentVel[1] < 0 and agentPos[1] < -enclosureSize:
                    agentPos[1] += 2 * enclosureSize
                elif agentVel[1] > 0 and agentPos[1] > enclosureSize:
                    agentPos[1] += -2 * enclosureSize

        vel_new = agentControllers[agent].vel(relevantPositions,relevantVels,agentPos,agentVel)

        # implement agent motion constraints
        corrected = False
        #angle deviation of velocity
        if(np.linalg.norm(vel_new) > 0 and np.linalg.norm(agentVel) > 0):
            vel_new_angle = np.arctan2(vel_new[1],vel_new[0])
            vel_angle = np.arctan2(agentVel[1],agentVel[0])
            angleDeviation = vel_new_angle-vel_angle
            if abs(angleDeviation) > maxAngleDeviation:
                # print("Correcting a deviation", angleDeviation,agentVel,vel_new)
                corrected = True
                agent_vel_hat = agentVel/np.linalg.norm(agentVel)
                agent_vel_complex = agent_vel_hat[0] + 1j*agent_vel_hat[1]
                agent_vel_complex *= np.linalg.norm(vel_new)
                # figure out which side to rotate
                if angleDeviation > 0:
                    agent_vel_complex *= cmath.exp(1j*-1*maxAngleDeviation)
                else:
                    agent_vel_complex *= cmath.exp(1j*maxAngleDeviation)
                vel_new = np.array([agent_vel_complex.real,agent_vel_complex.imag])
        
        # if corrected:
        #     print("Corrected", vel_new)
        #max acceleration
        if np.linalg.norm(vel_new)-np.linalg.norm(agentVel) > agentMaxVelChange:
            vel_new = vel_new*((agentMaxVelChange+np.linalg.norm(agentVel))/np.linalg.norm(vel_new))
        else:
            if np.linalg.norm(vel_new)-np.linalg.norm(agentVel) < -agentMaxVelChange:
                vel_new = vel_new*((-agentMaxVelChange+np.linalg.norm(agentVel))/np.linalg.norm(vel_new))

        #max vel
        if np.linalg.norm(vel_new) > agentMaxVel:
            vel_new *= (agentMaxVel/np.linalg.norm(vel_new))
        

        agentVels[step+1,agent] = vel_new
        agentPositions[step+1,agent] = vel_new*dt+agentPos

#graph with animation
# graphScale = 1.75
# axisBounds = graphScale*enclosureSize
# print(agentPositions)

#everything gets flattened into a 2d tabular frame, with frame as a category


df = pd.DataFrame(
    {
       "x":np.reshape(agentPositions[:,:,0],numAgents*(steps+1)),
       "y":np.reshape(agentPositions[:,:,1],numAgents*(steps+1)),
       "frame":np.repeat(range(0,steps+1),numAgents),
    }
)

print("Simulation complete--Generating media")

# scale is incorrect


if(interactive_graph):
    #interactive graph
    fig = px.scatter(df,x="x",y="y",animation_frame="frame")
    fig.update_layout(yaxis_range=[-enclosureSize,enclosureSize],xaxis_range=[-enclosureSize,enclosureSize])
    fig.update_layout(width=500,height=500)
    fig.show()

else: 
    if not os.path.exists("./images"):
        os.makedirs("./images")
    #save as gif
    def plot(i):
        d = df[df['frame'] == i]
        fig = px.scatter(d,x="x",y="y")
        fig.update_layout(yaxis_range=[-enclosureSize,enclosureSize],xaxis_range=[-enclosureSize,enclosureSize])
        fig.update_layout(width=500,height=500)
        fig.write_image("images/fig"+str(i)+".png")
        return fig

    frames = []

    writeToGif = False
    
    if writeToGif:
        for i in range(0,steps):
            plot(i)
            im = Image.open("images/fig"+str(i)+".png")
            frames.append(im)
        frames[0].save('overall.gif', format='GIF', append_images=frames[1:], save_all=True, duration=overallTime, loop=0)
    
    else:
        for i in range(0,steps):
            plot(i)
            # there may be a way to get opencv to 
            pil_image = Image.open("images/fig"+str(i)+".png")
            im = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
            frames.append(im)
        video = cv2.VideoWriter('overall_video.mp4',cv2.VideoWriter_fourcc(*'mp4v'), 1/dt, (500,500), True)

        for frame in frames:
            video.write(frame)
print("Media Generated!")