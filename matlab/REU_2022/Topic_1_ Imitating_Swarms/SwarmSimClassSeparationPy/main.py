import numpy as np
import plotly.express as px
import plotly.graph_objects as go
import pandas as pd
import LenardJones as lj
import Boids as bo
import gif
from PIL import Image

#toggles whether output to interactive graph of a gif
interactive_graph = True

# sim parameters
overallTime = 10
dt = .05
steps = int(overallTime/dt)
numAgents = 100
agentMaxVel = 5
neighborRadius = 5

# define arrays of positions and vels over each step
agentPositions = np.zeros([steps+1,numAgents,2])
agentVels = np.zeros([steps+1,numAgents,2])

#base inertia stuff broken, need to fix
agentControllers = [bo.Boids(1,1,.01,1) for i in range(numAgents)]

# initial positions and velocities
randPosMax = 75

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
        # print("Step "+str(step)+"Agent"+str(agent))
        # print(agentPos)
        agentVel = agentVels[step,agent]

        relevantPositions = []
        relevantVels = []

        #don't like passing global info
        for other_agent in range(0,numAgents):
            if agent == other_agent or np.linalg.norm(agentPos-agentPositions[step,other_agent]) > neighborRadius:
                continue
            relevantPositions.append(agentPositions[step,other_agent])
            relevantVels.append(agentVels[step,other_agent])


        agentVels[step+1,agent] = agentControllers[agent].vel(relevantPositions,relevantVels,agentPos,agentVel)
        if(np.linalg.norm(agentVels[step+1,agent]) > agentMaxVel):
            agentVels[step+1,agent] *= (agentMaxVel/np.linalg.norm(agentVels[step+1,agent]))
        # print(agentVels[step+1,agent])
        agentPositions[step+1,agent] = agentVels[step+1,agent]*dt+agentPos

#graph with animation
# graphScale = 1.75
# axisBounds = graphScale*randPosMax
# print(agentPositions)

#everything gets flattened into a 2d tabular frame, with frame as a category


df = pd.DataFrame(
    {
       "x":np.reshape(agentPositions[:,:,0],numAgents*(steps+1)),
       "y":np.reshape(agentPositions[:,:,1],numAgents*(steps+1)),
       "frame":np.repeat(range(0,steps+1),numAgents),
    }
)

# scale is incorrect


if(interactive_graph):
    #interactive graph
    fig = px.scatter(df,x="x",y="y",animation_frame="frame")
    fig.update_layout(yaxis_range=[-randPosMax,randPosMax],xaxis_range=[-randPosMax,randPosMax])

    fig.show()

else: 
    #save as gif
    def plot(i):
        d = df[df['frame'] == i]
        fig = px.scatter(d,x="x",y="y")
        fig.update_layout(width=500,height=500)
        fig.update_layout(yaxis_range=[-randPosMax,randPosMax],xaxis_range=[-randPosMax,randPosMax])
        fig.write_image("images/fig"+str(i)+".png")
        return fig

    frames = []

    for i in range(0,steps):
        plot(i)
        im = Image.open("images/fig"+str(i)+".png")
        frames.append(im)

    frames[0].save("overall.gif",format="GIF",append_images=frames,save_all=True,duration=overallTime,loop=0)