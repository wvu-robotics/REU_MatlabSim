function MainScriptFunction(ParamS, ParamC, ParamA, number)
% Main script: loads parameter variables and runs swarm step function
%% Clear
close all
%clear
clc

%% Add search paths for sim laws and agent functions
addpath("Code of Laws");
addpath("Agent Control Functions");

%% Load simulation parameters
simLaw = MaxsLaw();
simLaw.separation = ParamS;
simLaw.cohesion   = ParamC;
simLaw.alignment  = ParamA;

%% setup output folder
rootFolder = "Output Media";
dateFormat = "mm-dd-yy";
timeFormat = "HH-MM-SS";
%videoPrefix = "TestVideo";
%videoPrefix = sprintf('[dt %g, T %g, x%g] ',simLaw.dt, simLaw.totalTime, simLaw.fpsMult);
videoPrefix = "BIG";

% Get current date/time
date = datestr(now,dateFormat);
time = datestr(now,timeFormat);

% Create folder/video names
dateFolder = sprintf('%s/%s',rootFolder,date);


%videoSuffix = sprintf('S=%1.0E, C=%1.0E, A=%1.0E, M=%1.0E', simLaw.separation, simLaw.cohesion, simLaw.alignment, simLaw.migration);
%videoSuffix = time;
videoSuffix = sprintf('%1.0E, %1.0E, %1.0E', simLaw.separation, simLaw.cohesion, simLaw.alignment);
videoName = sprintf('%s/%s %s.avi',dateFolder,videoPrefix,videoSuffix);

% Create folders if they don't exist
if(~exist(rootFolder,'dir'))
    mkdir(rootFolder);
end

if(~exist(dateFolder,'dir'))
    mkdir(dateFolder);
end

%% Setup video and figure
video = VideoWriter(videoName);
video.FrameRate = 1/simLaw.dt * simLaw.fpsMult;
open(video);

simFig = figure('Visible','on');
xlim(simLaw.mapSize);
ylim(simLaw.mapSize);
daspect([1 1 1])

%% Create instance of simulation
swarm = Swarm(simLaw);
theta = linspace(0,2*pi,50);
patchX = 600*cos(theta)-1000;
patchY = 600*sin(theta)+1000;
patchObj = patch('XData',patchX,'YData',patchY,'FaceColor','red','FaceAlpha',0.8);

%% Run simulation
steps = simLaw.totalTime/simLaw.dt;
for step = 1:steps
%     c1 = clock;
    %fprintf("Frame %g/%g:  ",step,steps);
    
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
    %fprintf("%g Agents, ", Living);
    maxHeight = -1;
    minHeight = 1E6;
    averageHeight = 0;
    for i=1:simLaw.numAgents
        if swarm.agents(i).position(3) > 0
            currentHeight = swarm.agents(i).position(3);
            maxHeight = max(maxHeight,currentHeight);
            minHeight = min(minHeight,currentHeight);
            averageHeight = averageHeight + currentHeight;
        end
    end
    averageHeight = averageHeight / simLaw.numAgents;
    
    %stringTitle = sprintf("Agents Alive: %g\nMax Height: %.1f\nMin Height: %.1f\nAverage Height: %.1f",Living,maxHeight,minHeight,averageHeight);
    minutes = floor(step*simLaw.dt/60);
    %stringTitle = sprintf("Minutes: %g\nAgents Alive: %g\nAverage Height: %.1f",minutes,Living,averageHeight);
    stringTitle = sprintf("Number %g, Minute %g\nAgents Alive: %g  Average Height: %.1f\n S=%1.0E, C=%1.0E, A=%1.0E", ...
        number, minutes,Living,averageHeight,simLaw.separation, simLaw.cohesion, simLaw.alignment);
    title(stringTitle);

    % Find and print elapsed time
%     c2 = clock;
%     elapsedTime = c2(6)-c1(6);
    % If minute advances, elapsedTime will appear negative (1min20sec - 0min50sec = 20sec-50sec = -30sec)
%     if(elapsedTime < 0) 
%         elapsedTime = elapsedTime + 60;
%     end
    %fprintf("%g sec\n",elapsedTime);
end

close(video);
end