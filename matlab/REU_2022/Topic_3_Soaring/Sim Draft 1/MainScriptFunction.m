function [average, surviving] = MainScriptFunction(ParamS, ParamC, ParamA, ParamM, ParamhPr, ParamhIg, dt, number, render)
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
simLaw.migration  = ParamM;
simLaw.heightPriority = ParamhPr;
simLaw.heightIgnore   = ParamhIg;
simLaw.dt         = dt;

%% Video Initialization
if render
    %% setup output folder
    dateFormat = "mm-dd-yy";
    timeFormat = "HH-MM-SS";
    % Get current date/time
    date = datestr(now,dateFormat);
    time = datestr(now,timeFormat);
    rootFolder = "Output Media";
    % Create folder/video names
    dateFolder = sprintf('%s/%s',rootFolder,date);

    % Create folders if they don't exist
    if(~exist(rootFolder,'dir'))
        mkdir(rootFolder);
    end
    if(~exist(dateFolder,'dir'))
        mkdir(dateFolder);
    end

    %% Setup video and figure
%     videoPrefix = sprintf('[dt %g, T %g, x%g] ',simLaw.dt, simLaw.totalTime, simLaw.fpsMult);
    videoPrefix = "BIG";
%     videoSuffix = time;
    videoSuffix = sprintf('%1.0E, %1.0E, %1.0E', simLaw.separation, simLaw.cohesion, simLaw.alignment);
    videoName = sprintf('%s/%s %s.avi',dateFolder,videoPrefix,videoSuffix);

    video = VideoWriter(videoName);
    video.FrameRate = 1/simLaw.dt * simLaw.fpsMult;
    open(video);
    
    simFig = figure('Visible','on');
    xlim(simLaw.mapSize);
    ylim(simLaw.mapSize);
    daspect([1 1 1])
end

%% Create Swarm and Render Thermal
swarm = Swarm(simLaw);
if render
    theta  = linspace(0,2*pi,50);
    patchX = 600*cos(theta)-1000;
    patchY = 600*sin(theta)+1000;
    patchObj = patch('XData',patchX,'YData',patchY,'FaceColor','red','FaceAlpha',0.8);
end

%% Run simulation...
steps = simLaw.totalTime/simLaw.dt;
Living = simLaw.numAgents;
maxHeight = -1;
minHeight = 1E6;
averageHeight = 0;
c1 = clock;
for step = 1:steps
    %% Step simulation & Get Data
    swarm.saveAgentData();
    swarm.stepSimulation();
    for i=1:simLaw.numAgents
        if swarm.agents(i).position(3) > 0
            currentHeight = swarm.agents(i).position(3);
            maxHeight = max(maxHeight,currentHeight);
            minHeight = min(minHeight,currentHeight);
            averageHeight = averageHeight + currentHeight;
        end
    end
    averageHeight = averageHeight / simLaw.numAgents;
    minutes = floor(step*simLaw.dt/60);
    Living = nnz([swarm.agents.isAlive]);

    %% Render
    if render
        swarm.renderAgents();
        currFrame = getframe(simFig);
        writeVideo(video,currFrame);
        pause(0.0001);

%         stringTitle = sprintf("Agents Alive: %g\nMax Height: %.1f\nMin Height: %.1f\nAverage Height: %.1f",Living,maxHeight,minHeight,averageHeight);
%         stringTitle = sprintf("Minutes: %g\nAgents Alive: %g\nAverage Height: %.1f",minutes,Living,averageHeight);
        stringTitle = sprintf("Number %g, Minute %g\nAgents Alive: %g  Average Height: %.1f\n S=%1.0E, C=%1.0E, A=%1.0E", ...
            number, minutes,Living,averageHeight,simLaw.separation, simLaw.cohesion, simLaw.alignment);
        title(stringTitle);
    end
    
    %% Print and Advance Clock
    if mod(step,100) == 0
        c2 = clock;
        dTime = c2(6)-c1(6);
        if(dTime < 0) 
            %If minute advances, elapsedTime will appear negative (1min20sec - 0min50sec = 20sec-50sec = -30sec)
            dTime = dTime + 60;
        end
        fprintf("Frame %g/%g:  ",step,steps);
        fprintf("Run # %g, ", number);
        fprintf("%g Agents, ", Living);
        fprintf("Minute %g, ", minutes);
        fprintf("%g sec\n",dTime);
        c1 = clock;
    end
end
average = averageHeight;
surviving = Living;
if render 
    close(video); 
end
end