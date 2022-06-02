import pydtmc
from pydtmc import MarkovChain 
import numpy as np
import plotly.express as px
import pandas as pd

# sim parameters
overallTime = 5
dt = .1
steps = int(overallTime/dt)
numAgents = 100
agentMaxVel = 10
radius = 5

# define arrays of positions and vels over each step
agentPositions = np.zeros([steps+1,numAgents,2])
agentVels = np.zeros([steps+1,numAgents,2])

# initial positions and velocities
randPosMax = 100

for agentPos in agentPositions[0]:
    #probably a way to assign whole array at once, but this is more explicit
    agentPos[0] = np.random.uniform(-randPosMax,randPosMax)  
    agentPos[1] = np.random.uniform(-randPosMax,randPosMax)

for agentVel in agentPositions[0]:
    agentVel[0] = np.random.uniform(-agentMaxVel,agentMaxVel)  
    agentVel[1] = np.random.uniform(-agentMaxVel,agentMaxVel)

# define states
stateA = agentVel*3
stateB = agentVel*-3
stateC = agentVel
stateD = agentVel+2

p = [[0.30, 0.30, 0.40, 0.00], [0.15, 0.25, 0.35, 0.25], [0.20, 0.30, 0.30, 0.20], [0.50, 0.00, 0.40, 0.10]]
mc = MarkovChain(p, ['A', 'B', 'C', 'D'])

#run simulation
for step in range(0,steps):
   for agent in range(0,numAgents):
        walk = pydtmc.MarkovChain.walk(self=mc, steps=steps, initial_state='A', final_state=None, output_indices=False, seed=None)
        agentPos = agentPositions[step,agent]
        agentVel = agentVels[step,agent]
        if walk[step] == 'A':
            agentVels[step+1,agent]= stateA
            agentPositions[step+1,agent] = agentVels[step+1,agent]*dt+agentPos
        elif walk[step] == 'B':
            agentVels[step+1,agent]= stateB
            agentPositions[step+1,agent] = agentVels[step+1,agent]*dt+agentPos
        elif walk[step] == 'C':
            agentVels[step+1,agent]= stateC
            agentPositions[step+1,agent] = agentVels[step+1,agent]*dt+agentPos
        else:
            agentVels[step+1,agent]= stateD
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
fig = px.scatter(df,x="x",y="y",animation_frame="frame")
fig.show()