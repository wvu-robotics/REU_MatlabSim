function [data, average, surviving, flightTime, Log] = MainScriptFunction(Param, render)
% Main script: loads parameter variables and runs swarm step function
%% Clear
close all
%clear
%clc

%% Add search paths for sim laws and agent functions
addpath("Code of Laws");
addpath("Agent Control Functions");
addpath("Find Neighborhood Functions");

%% Load simulation parameters
SL = MaxsLaw();
% Initialize thermals as a matrix of Thermals
thermalMap = ThermalMap(SL);

number                  = Param(1);
SL.separation           = 10^Param(2);
SL.cohesion             = 10^Param(3);
SL.alignment            = 10^Param(4);
SL.migration            = 10^Param(5);
SL.cohesionHeightMult   = Param(6);
SL.separationHeightGap  = Param(7);
SL.dt                   = Param(8);
SL.waggle               = Param(9);
SL.waggleTime           = Param(10);
SL.numAgents            = Param(11);
SL.thermalStrengthMin   = Param(12);
SL.thermalStrengthMax   = Param(13);
SL.forwardSpeedMin      = Param(14);
SL.forwardSpeedMax      = Param(15);

swarm = Swarm(SL, thermalMap);


%% Video Initialization...
dateFormat = "mm-dd-yy";
timeFormat = "HH-MM-SS";
% Get current date/time
date = datestr(now,dateFormat);
time = datestr(now,timeFormat);

if render
    %% setup output folder
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
    videoPrefix = "A";
    %videoSuffix = sprintf('%02g',Param(1));
    videoSuffix = time;
%    videoSuffix = sprintf('%1.0E, %1.0E, %1.0E', simLaw.separation, simLaw.cohesion, simLaw.alignment);
    videoName = sprintf('%s/%s %s.avi',dateFolder,videoPrefix,videoSuffix);
    
    swarm.initVideo(videoName);
end

%% Run simulation...
steps  = SL.totalTime/SL.dt;
Living = SL.numAgents;
averageHeight = 0;
data = zeros(3,steps/SL.frameSkip);
flightTime          = 0;
% ToD           = zeros(1,SL.numAgents);

for step = 1:steps
    if ~SL.stopWhenDead || nnz([swarm.agents.isAlive]) > 0
        %% Step simulation
        thermalMap.staticStep();
        swarm.saveAgentData();
        swarm.stepSimulation();

        
        if mod(step,SL.frameSkip)==0
            %% Read Data
            swarm.updateData(step);
            data(1,step/SL.frameSkip) = swarm.maxHeight;
            data(2,step/SL.frameSkip) = swarm.minHeight;
            data(3,step/SL.frameSkip) = swarm.avgHeight;
            %% Render
            if render
                swarm.renderAll
            end
        end

        %% Print
        if mod(step,steps/10) == 0
            fprintf("%02g%% through Run #  %g \n",100*step/steps, number);
        end
    else
        fprintf("Everybody died in Run # %g\n", number);
        break
    end
end
if render 
    swarm.closeVideo(); 
end
%% Write to Log

% Date
% Time

% Separation
% Cohesion
% Alignment	
% Migration
% Height-Priority
% Height-Ignore
% Waggle-Strength	
% Waggle-Time
% K of KNN (Finding Neighborhood)

% dt
% Total-Time
% # of Agents
% # of Thermals
% Neighbor-Radius
% FOV
% Control Function
% Neighborhood Function
% Speed Min
% Speed Max
% Thermal Strength Min
% Thermal Strength Max

% Surviving
% Average Z of survivors
% Flight Time (Score)
   

timeFormat2 = "HH:MM:SS";
time = datestr(now,timeFormat2);

Log = {date, time,...
       SL.separation, SL.cohesion, SL.alignment,SL.migration,...
       SL.cohesionHeightMult, SL.separationHeightGap, SL.waggle, SL.waggleTime, SL.k,...
       SL.dt, SL.totalTime,SL.numAgents, SL.numThermals, SL.neighborRadius,...
       SL.fov, SL.funcName_agentControl, SL.funcName_findNeighborhood, SL.forwardSpeedMin, SL.forwardSpeedMax,...
       SL.thermalStrengthMin, SL.thermalStrengthMax, surviving, average, flightTime};


end