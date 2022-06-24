function [average, surviving] = MainScriptFunction(Param, render)
% Main script: loads parameter variables and runs swarm step function
%% Clear
close all
%clear
clc

%% Add search paths for sim laws and agent functions
addpath("Code of Laws");
addpath("Agent Control Functions");
addpath("Find Neighborhood Functions");

%% Load simulation parameters
simLaw = MaxsLaw();
% Initialize thermals as a matrix of Thermals
thermalMap = ThermalMap(simLaw, 200, 0);

number                      = Param(1);
simLaw.separation           = 10^Param(2);
simLaw.cohesion             = 10^Param(3);
simLaw.alignment            = 10^Param(4);
simLaw.migration            = 10^Param(5);
simLaw.cohesionHeightMult   = Param(6);
simLaw.separationHeightGap  = Param(7);
simLaw.dt                   = Param(8);
simLaw.waggle               = Param(9);
simLaw.waggleTime           = Param(10);
simLaw.numAgents            = Param(11);

%% Video Initialization...
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
    videoSuffix = time;
%    videoSuffix = sprintf('%1.0E, %1.0E, %1.0E', simLaw.separation, simLaw.cohesion, simLaw.alignment);
    videoName = sprintf('%s/%s %s.avi',dateFolder,videoPrefix,videoSuffix);

    video = VideoWriter(videoName);
    video.FrameRate = 1/simLaw.dt * simLaw.fpsMult;
    open(video);
    
    simFig = figure('Visible','on');

    % Initialize map background
    clf
    xlim(simLaw.mapSize);
    ylim(simLaw.mapSize);
    daspect([1 1 1]);
    colorbar;
    cbLimits = [-1,simLaw.thermalStrengthMax];
    colors = [6 42 127; 41 76 247; 102 59 231; 162 41 216; 222 24 200; 255 192 203] / 255;
    x = [0:thermalMap.thermalPixels/(length(colors)-1):thermalMap.thermalPixels];
    map = interp1(x/thermalMap.thermalPixels,colors,linspace(0,1,thermalMap.thermalPixels)); % Creates a color gradient for the map
    colormap();%map);
    set(gca,'clim',cbLimits);
end

%% Run simulation...
% Render Thermal

% if render
%     theta  = linspace(0,2*pi,50);
%     patchX = 600*cos(theta)-1000;
%     patchY = 600*sin(theta)+1000;
%     patchObj = patch('XData',patchX,'YData',patchY,'FaceColor','red','FaceAlpha',0.8);
% end
swarm = Swarm(simLaw);
steps = simLaw.totalTime/simLaw.dt;
Living = simLaw.numAgents;
maxHeight = -1;
minHeight = 1E6;
averageHeight = 0;

for step = 1:steps
    %% Step simulation & Get Data
    thermalMap.staticStep();

    swarm.saveAgentData();
    swarm.stepSimulation(thermalMap);
    averageHeight = 0;
    for i=1:simLaw.numAgents
        if swarm.agents(i).isAlive
            currentHeight = swarm.agents(i).position(3);
            maxHeight = max(maxHeight,currentHeight);
            minHeight = min(minHeight,currentHeight);
            averageHeight = averageHeight + currentHeight;
        end
    end
    averageHeight = averageHeight / nnz([swarm.agents.isAlive]);
    minutes = floor(step*simLaw.dt/60);
    Living = nnz([swarm.agents.isAlive]);


    
    %% Render
    if render && mod(step,simLaw.frameSkip)==1
        hold on
        finalThermalMap = thermalMap.renderThermals();
        if step == 1
            thermalMapImg = imagesc(finalThermalMap,'XData',simLaw.mapSize,'YData',simLaw.mapSize);
            thermalMapImg.AlphaData = 1;
        else 
            thermalMapImg.CData = finalThermalMap;
        end
        swarm.renderAgents();
        currFrame = getframe(simFig);
        writeVideo(video,currFrame);
        pause(0.0001);

%         stringTitle = sprintf("Agents Alive: %g\nMax Height: %.1f\nMin Height: %.1f\nAverage Height: %.1f",Living,maxHeight,minHeight,averageHeight);
%         stringTitle = sprintf("Minutes: %g\nAgents Alive: %g\nAverage Height: %.1f",minutes,Living,averageHeight);
        stringTitle = sprintf("Number %g, Minute %g\nAgents Alive: %g  Average Height: %.1f", ...
            number, minutes,Living,averageHeight);
        title(stringTitle);
        hold off
    end
    
    %% Print
    if mod(step,100) == 0
        fprintf("%02.2f%%, ",100*step/steps);
        fprintf("Run # %g \n", number);
        % fprintf("%g Agents\n ", Living);
    end
end
average = averageHeight;
surviving = Living;
if render 
    close(video); 
end
end