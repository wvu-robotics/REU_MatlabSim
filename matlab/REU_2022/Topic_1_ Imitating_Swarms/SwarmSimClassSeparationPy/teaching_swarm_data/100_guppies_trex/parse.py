# parse data into a more useful npz/similar
# needed for output parse - enclosure size(hopefully square), num_agents, agentPositions, steps, overall_time
# agentPositions = np.zeros()
from time import time
from matplotlib.pyplot import step
import numpy as np
from functools import reduce

names = ["video_example_100fish_1min_fish"+str(i)+".npz" for i in range(0, 100)]

# scale pixels to a given scale, depending on unit
scale = 1

max_edge = 0  # max of any component
num_agents = 100

steps = 1900 # artificially choosing to cut off a few, since tracking data breaks
agentPositions = np.zeros([steps, num_agents, 2])
overall_time = np.load(names[0])['time'][-1]

max_component = 0.0 # used to derive square enclosure size
for i in range(0, 100):
    datazip = np.load(names[i])
    # they also 
    x = datazip['X']
    y = datazip['Y']

    #ignores infs
    def safe_maximum(a,b):
        if a == np.inf:
            if b == np.inf: return 0
            else: return b
        if b == np.inf:
            if b == np.inf: return 0
            else: return b
        return np.maximum(a,b)
    def safe_max(a):
        return reduce(safe_maximum,a)

    max_component = np.max([max_component,safe_max(x),safe_max(y)])

    # print("X",x)
    # print("Y",y)
    # print("Times",datazip['time'])


    points = np.array([x[:steps],y[:steps]]).transpose()
    # print("Points",points)
    agentPositions[:,i,:] = points

# recenter agent positions(again assumes square, not generalizable)
agentPositions = agentPositions - max_component/2

# filter out infs, replace with last seen positions
for s in range(1,steps):
    for agent in range(num_agents):
        if  np.bitwise_not(np.isfinite(agentPositions[s][agent])).any():
            agentPositions[s][agent] = agentPositions[s-1][agent]

encSize = max_component*1.1/2


nthFrame = 20
slimmedPositions = []
for i in range(len(agentPositions)):
    if i%nthFrame ==0:
        slimmedPositions.append(agentPositions[i])

agentPositions = np.array(slimmedPositions)

steps = len(agentPositions)

print("Agent Positions",agentPositions)
print("Overall Time",overall_time)
print("Steps",steps)
print("Num Agents",num_agents)
print("Enclosure Size",encSize)



np.savez('100fish_parsed_every'+str(nthFrame)+'th_frame',agent_positions=agentPositions,overall_time=overall_time,steps=steps,num_agents=num_agents,enclosure_size=encSize)