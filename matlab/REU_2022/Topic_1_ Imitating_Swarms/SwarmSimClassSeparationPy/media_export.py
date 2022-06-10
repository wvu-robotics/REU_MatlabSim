import numpy as np
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
from PIL import Image
import cv2
import os
import sim
import enum
import io
from tqdm import tqdm

class ExportType(enum.Enum):
    NONE = 0
    INTERACTIVE = 1
    GIF = 2
    MP4 = 3

def export(export_type,name,agentPositions,params=sim.SimParams(),controllers = [],vision_mode=False,progress_bar=False):
    #controllers used to grab per-agent color
    colors = [controller.color for controller in controllers]
    
    if export_type == ExportType.INTERACTIVE:
        interactive(agentPositions,params,colors)
    elif export_type == ExportType.GIF:
        toGIF(name,agentPositions,params,colors,vision_mode=vision_mode,progress_bar=progress_bar)
    elif export_type == ExportType.MP4:
        toMP4(name,agentPositions,params,colors,vision_mode=vision_mode,progress_bar=progress_bar)
    else:
        pass

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

def toGIF(name,agentPositions,params=sim.SimParams(),colors=[],vision_mode=False,progress_bar=False):
    df= toPandasFrame(agentPositions,colors=colors,params=params)
    frames =[]
    for i in (tqdm(range(0,len(agentPositions))) if progress_bar else range(0,len(agentPositions))):
        fig = plot(i,df,params,vision_mode,write=False)
        fig_bytes = fig.to_image(format='png') #getting rid of file writes,slightly faster
        buf = io.BytesIO(fig_bytes)
        im = Image.open(buf)
        frames.append(im)
    frames[0].save(name+'.gif', format='GIF', append_images=frames[1:], save_all=True, duration=params.overall_time, loop=0)

def toMP4(name,agentPositions,params=sim.SimParams(),colors=[],vision_mode=False,progress_bar=False):
    df= toPandasFrame(agentPositions,colors=colors,params=params)
    video = cv2.VideoWriter(name+'.mp4',cv2.VideoWriter_fourcc(*'mp4v'), 1/params.dt, (500,500), True)
    for i in (tqdm(range(0,len(agentPositions))) if progress_bar else range(0,len(agentPositions))):
        fig = plot(i,df,params,vision_mode,write=False)
        fig_bytes = fig.to_image(format='png') #getting rid of file writes,slightly faster
        buf = io.BytesIO(fig_bytes)
        img = Image.open(buf) #would like a way to go from bytes to cv2 image immediately, haven't found yet
        im = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
        video.write(im)