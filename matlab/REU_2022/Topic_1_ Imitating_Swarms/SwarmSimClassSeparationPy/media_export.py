import numpy as np
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
from PIL import Image
import cv2
import os
import sim
import enum

class ExportType(enum.Enum):
    NONE = 0
    INTERACTIVE = 1
    GIF = 2
    MP4 = 3

def export(export_type,name,agentPositions,params=sim.SimParams(),vision_mode=False):
    if export_type == ExportType.INTERACTIVE:
        interactive(agentPositions,params)
    elif export_type == ExportType.GIF:
        toGIF(name,agentPositions,params,vision_mode)
    elif export_type == ExportType.MP4:
        toMP4(name,agentPositions,params,vision_mode)
    else:
        pass

def toPandasFrame(agentPositions,params=sim.SimParams()):
    steps = int(params.overall_time/params.dt)
    return pd.DataFrame(
    {
       "x":np.reshape(agentPositions[:,:,0],params.num_agents*(steps+1)),
       "y":np.reshape(agentPositions[:,:,1],params.num_agents*(steps+1)),
       "frame":np.repeat(range(0,steps+1),params.num_agents),
    }
)

def interactive(agentPositions,params=sim.SimParams()):
    df= toPandasFrame(agentPositions,params)
    fig = px.scatter(df,x="x",y="y",animation_frame="frame")
    fig.update_layout(yaxis_range=[-params.enclosure_size,params.enclosure_size],xaxis_range=[-params.enclosure_size,params.enclosure_size])
    fig.update_layout(width=500,height=500)
    fig.show()


def plot(i,df,params=sim.SimParams(),vision_mode=False):
    d = df[df['frame'] == i]
    fig = px.scatter(d,x="x",y="y")
    fig.update_layout(yaxis_range=[-params.enclosure_size,params.enclosure_size],xaxis_range=[-params.enclosure_size,params.enclosure_size])
    fig.update_layout(width=500,height=500)
    if vision_mode == True:
        fig.update_layout(template="plotly_dark")
        fig.update_layout(xaxis_title="", yaxis_title="")
        fig.update_xaxes(showgrid=False,zeroline=False, showticklabels=False)
        fig.update_yaxes(showgrid=False,zeroline=False, showticklabels=False)
        fig.update_traces(marker=dict(size=8))
    fig.write_image("frames/fig"+str(i)+".png")
    return fig

def toGIF(name,agentPositions,params=sim.SimParams(),vision_mode=False):
    df= toPandasFrame(agentPositions,params)
    if not os.path.exists("./frames"):
        os.makedirs("./frames")
    
    frames = []
    
    for i in range(0,len(agentPositions)):
        plot(i,df,params,vision_mode)
        im = Image.open("frames/fig"+str(i)+".png")
        frames.append(im)
    frames[0].save(name+'.gif', format='GIF', append_images=frames[1:], save_all=True, duration=params.overall_time, loop=0)

def toMP4(name,agentPositions,params=sim.SimParams(),vision_mode=False):
    df= toPandasFrame(agentPositions,params)
    if not os.path.exists("./frames"):
        os.makedirs("./frames")
    
    frames = []
    
    for i in range(0,len(agentPositions)):
        plot(i,df,params.enclosure_size,vision_mode)
        pil_image = Image.open("frames/fig"+str(i)+".png")
        im = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
        frames.append(im)
    video = cv2.VideoWriter(name+'.mp4',cv2.VideoWriter_fourcc(*'mp4v'), 1/params.dt, (500,500), True)
    for frame in frames:
        video.write(frame)





