import numpy as np
import plotly.express as px
import pandas as pd

# sim parameters
overallTime = 10
dt = .1
steps = int(overallTime/dt)
numAgents = 100
agentMaxVel = 3
neighborRadius = 5

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

# define velocity function
def v(positions,src,v_0,neighborRadius=np.Inf,agentMaxVel=np.Inf):
    v_gain = np.zeros(2)

    #find distance vectors for all nearby
    for position in positions:
        # print("Position")
        # print(position)
        # print("Src")
        # print(src)
        diffPos = position-src
        dist = np.linalg.norm(diffPos)
        if dist == 0 or dist > neighborRadius:
            continue
        unit_diff = diffPos / dist

        #non linearities

        #lennard-jones, actually works
        epsilon = 100 
        sigma = 1
        out = epsilon*(((sigma/dist)**12)+(-2*((sigma/dist)**6)))

        if out != 0:
            v_gain += out*unit_diff
    inertia = 1
    v_out = (v_0*inertia) + v_gain
    v_out_mag = np.linalg.norm(v_out)
    if v_out_mag > agentMaxVel:
        v_out *= (agentMaxVel/v_out_mag)
    return v_out
        
#run simulation
for step in range(0,steps):
   for agent in range(0,numAgents):
        agentPos = agentPositions[step,agent]
        agentVel = agentVels[step,agent]

        agentVels[step+1,agent]= v(agentPositions[step],agentPos,agentVel,neighborRadius,agentMaxVel)
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