% Main script: loads parameter variables and runs swarm step function
%% Clear
close all
clear
clc

%% setup output folder
rootFolder = "Output Media";
dateFormat = "mm-dd-yy";
timeFormat = "HH-MM-SS";
videoPrefix = "TestVideo";

% Get current date/time
date = datestr(now,dateFormat);
time = datestr(now,timeFormat);

% Create folder/video names
dateFolder = sprintf('%s/%s',rootFolder,date);
videoName = sprintf('%s/%s %s.avi',dateFolder,videoPrefix,time);

% Create folders if they don't exist
if(~exist(rootFolder,'dir'))
    mkdir(rootFolder);
end

if(~exist(dateFolder,'dir'))
    mkdir(dateFolder);
end

%% Setup video and figure
video = VideoWriter(videoName);
video.FrameRate = 1/SimLaw.dt;
open(video);

simFig = figure('Visible','on');
xlim(SimLaw.mapSize);
ylim(SimLaw.mapSize);
daspect([1 1 1])

%% Create instance of simulation
swarm = Swarm();

%% Run simulation
steps = SimLaw.getSteps();
for step = 1:steps
    c1 = clock;
    fprintf("Frame %g/%g:  ",step,steps);
    
    % Render agents
    swarm.renderAgents();
    
    % Save video frame
    currFrame = getframe(simFig);
    writeVideo(video,currFrame);
    pause(0.0001);
    
    % Step simulation
    swarm.saveAgentData();
    swarm.stepSimulation();
    
    % Print number of Living Agents
    Living = nnz([swarm.agents.isAlive]);
    fprintf("%g Agents, ", Living);

    % Find and print elapsed time
    c2 = clock;
    elapsedTime = c2(6)-c1(6);
    % If minute advances, elapsedTime will appear negative (1min20sec - 0min50sec = 20sec-50sec = -30sec)
    if(elapsedTime < 0) 
        elapsedTime = elapsedTime + 60;
    end
    fprintf("%g sec\n",elapsedTime);
end

close(video);