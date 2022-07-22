import numpy as np
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
from PIL import Image
import cv2
import os
import sim_tools.sim as sim
import enum
import io
from tqdm import tqdm
import imageio
import multiprocessing as mp
from functools import partial

class ExportType(enum.Enum):
    NONE = 0
    INTERACTIVE = 1
    GIF = 2
    MP4 = 3
    CVS=4

def export(export_type,name,agentPositions,agentVels,params=sim.SimParams(),controllers = [],vision_mode=False,progress_bar=False,framecap=20):
    #controllers used to grab per-agent color
    colors = [controller.color for controller in controllers]
    
    if export_type == ExportType.INTERACTIVE:
        interactive(agentPositions,params,colors)
    elif export_type == ExportType.GIF:
        toGIF(name,agentPositions,params,colors,vision_mode=vision_mode,progress_bar=progress_bar)
    elif export_type == ExportType.MP4:
        toMP4(name,agentPositions,params,colors,vision_mode=vision_mode,progress_bar=progress_bar,framecap=framecap)
    elif export_type == ExportType.CVS:
        toCVS(name,agentPositions,agentVels,params)
    else:
        pass

# breaking interface just for this particular kind of graph
def exportTrajectories(name:str, agentPositions,params:sim.SimParams,startTime:int,duration:int,size=800):
    start = int(startTime/params.dt)
    num_frames = int(duration/params.dt)
    agentPositionsSamples = agentPositions[start:start+num_frames]
    steps = len(agentPositionsSamples)-1
    df = pd.DataFrame(
        {
           "x":np.reshape(agentPositionsSamples[:,:,0],params.num_agents*(steps+1)),
           "y":np.reshape(agentPositionsSamples[:,:,1],params.num_agents*(steps+1)),
           "agent": np.tile(range(0,params.num_agents),steps+1)
        }
    )

    fig = px.line(df,x="x",y="y",color="agent",title="Agent Trajectories")
    fig.update_layout(yaxis_range=[-params.enclosure_size,params.enclosure_size],xaxis_range=[-params.enclosure_size,params.enclosure_size])
    fig.update_layout(width=size,height=size)
    fig.write_image(file = name+".png")


def toPandasFrame(agentPositions,colors=[],params=sim.SimParams()):
    steps = int(params.overall_time/params.dt)
    if colors == []:
        colors = np.repeat('rgb(99, 110, 250)',(steps+1)*params.num_agents)
    else:
        colors = np.tile(colors,(steps+1)) #should already be num_agents long
    return pd.DataFrame(
    {
       "x":np.reshape(agentPositions[:,:,0],params.num_agents*(steps+1)),
       "y":np.reshape(agentPositions[:,:,1],params.num_agents*(steps+1)),
       "frame":np.repeat(range(0,steps+1),params.num_agents),
       "color":colors
    }
)

def interactive(agentPositions,params=sim.SimParams(),colors=[]):
    df= toPandasFrame(agentPositions,colors=colors,params=params)
    fig = px.scatter(df,x="x",y="y",animation_frame="frame")
    fig.update_layout(yaxis_range=[-params.enclosure_size,params.enclosure_size],xaxis_range=[-params.enclosure_size,params.enclosure_size])
    fig.update_layout(width=500,height=500)
    fig.show()


def plot(i,df,params=sim.SimParams(),vision_mode=False,write=False):
    d = df[df['frame'] == i]

    fig = go.Figure(data=[go.Scatter(x=d["x"],y=d["y"],mode="markers",marker=dict(color=d["color"]))])
    fig.update_layout(yaxis_range=[-params.enclosure_size,params.enclosure_size],xaxis_range=[-params.enclosure_size,params.enclosure_size])
    fig.update_layout(showlegend=False)
    fig.update_layout(width=500,height=500)
    if vision_mode == True:
        fig.update_layout(template="plotly_dark")
        fig.update_layout(xaxis_title="", yaxis_title="")
        fig.update_xaxes(showgrid=False,zeroline=False, showticklabels=False)
        fig.update_yaxes(showgrid=False,zeroline=False, showticklabels=False)
        fig.update_traces(marker=dict(size=8))
    if write: fig.write_image("frames/fig"+str(i).zfill(5)+".png")
    return fig

def plotToBytes(i,df,params=sim.SimParams(),vision_mode=False,write=False):
    fig = plot(i,df,params,vision_mode,write)
    fig_bytes = fig.to_image(format='png')
    buf = io.BytesIO(fig_bytes)
    img = Image.open(buf)
    im = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
    return im

# can also be multiprocessed
def toGIF(name,agentPositions,params=sim.SimParams(),colors=[],vision_mode=False,progress_bar=False):
    df= toPandasFrame(agentPositions,colors=colors,params=params)
    maxGifRate = 20
    frames =[]
    if (1/params.dt) < maxGifRate:
        for i in (tqdm(range(0,len(agentPositions))) if progress_bar else range(0,len(agentPositions))):
            fig = plot(i,df,params,vision_mode,write=False)
            fig_bytes = fig.to_image(format='png') #getting rid of file writes,slightly faster
            buf = io.BytesIO(fig_bytes)
            img = Image.open(buf)
            im = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
            frames.append(img)
    else:
        for i in (tqdm(range(0,len(agentPositions))) if progress_bar else range(0,len(agentPositions))):
            n_sample_frame = int(1/(params.dt*maxGifRate))
            if (i%n_sample_frame ==0):
                fig = plot(i,df,params,vision_mode,write=False)
                fig_bytes = fig.to_image(format='png') #getting rid of file writes,slightly faster
                buf = io.BytesIO(fig_bytes)
                img = Image.open(buf)
                im = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
                frames.append(img)
    imageio.mimsave(name+'.gif',frames,fps = min(1/params.dt,maxGifRate)) # implement fps capping if this is good

# should be able to multithread the frame creations
def toMP4(name,agentPositions,params=sim.SimParams(),colors=[],vision_mode=False,progress_bar=False,framecap=60,threads=8):
    pool = mp.Pool(threads)
    df= toPandasFrame(agentPositions,colors=colors,params=params)


    video = cv2.VideoWriter(name+'.mp4',cv2.VideoWriter_fourcc(*'mp4v'), min(1/params.dt,framecap), (500,500), True)

    n_sample_frame = 1
    if (1/params.dt) < framecap: n_sample_frame = int(1/(params.dt*framecap))    
    # next step is to try writes to files, and see if that's faster bc less memory passing around

    # might be more speed related to minimizing size of dfs passed to each process
    ims = pool.map(partial(plotToBytes,df=df,params=params,vision_mode=vision_mode,write=False),range(0,len(agentPositions),n_sample_frame))

    for im in ims:
        video.write(im)

    # for i in range(0,len(agentPositions),n_sample_frame):
    #     im = Image.open("frames/fig"+str(i).zfill(5)+".png")
    #     im = cv2.cvtColor(np.array(im), cv2.COLOR_RGB2BGR)
    #     video.write(im)

    # if (1/params.dt) < framecap:
    #     for i in (tqdm(range(0,len(agentPositions))) if progress_bar else range(0,len(agentPositions))):
    #         fig = plot(i,df,params,vision_mode,write=False)
    #         fig_bytes = fig.to_image(format='png') #getting rid of file writes,slightly faster
    #         buf = io.BytesIO(fig_bytes)
    #         img = Image.open(buf) #would like a way to go from bytes to cv2 image immediately, haven't found yet
    #         im = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
    #         video.write(im)
    # else:
    #     n_sample_frame = int(1/(params.dt*framecap)) # neeed to think about this math, there may be a better way to interp
    #     for i in (tqdm(range(0,len(agentPositions))) if progress_bar else range(0,len(agentPositions))):
    #         if (i%n_sample_frame ==0):
    #             fig = plot(i,df,params,vision_mode,write=False)
    #             fig_bytes = fig.to_image(format='png') #getting rid of file writes,slightly faster
    #             buf = io.BytesIO(fig_bytes)
    #             img = Image.open(buf) #would like a way to go from bytes to cv2 image immediately, haven't found yet
    #             im = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
    #             video.write(im)

def toCVS(name,agentPositions,agentVels,params=sim.SimParams()):
    path="./data/"
    filename=path+name

    if not os.path.exists(path):
        os.makedirs(path)
    
    if os.path.exists(filename):
        os.remove(filename)
    
    steps = int(params.overall_time/params.dt)
    data=pd.DataFrame(
    {
       "x":np.reshape(agentPositions[:,:,0],params.num_agents*(steps+1)),
       "y":np.reshape(agentPositions[:,:,1],params.num_agents*(steps+1)),
       "vx":np.reshape(agentVels[:,:,0],params.num_agents*(steps+1)),
       "vy":np.reshape(agentVels[:,:,1],params.num_agents*(steps+1)),
       "frame":np.repeat(range(0,steps+1),params.num_agents),
    }
    )
   
    data.to_csv(filename)