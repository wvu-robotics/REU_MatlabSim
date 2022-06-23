% Thermal testing file for time-varying and location-varying thermal map
close all
clear
clc

%% Load sim law
addpath("Code of Laws");
addpath("Agent Control Functions");
addpath("Find Neighborhood Functions");
SL = ThermalTestLaw();
simLaw = ColesLaw();

%% Set up parameters
overallTime = 60; % s
dt = .05; % s 

% Initialize thermals as a matrix of Thermals
thermalMap = ThermalMap(SL);

%% Set up video and figure
video = VideoWriter('Thermal Outputs/thermalsWithBoids.avi');
video.FrameRate = 1/simLaw.dt * simLaw.fpsMult;
open(video);

simFig = figure('Visible','on');

%% Initialize map background
clf
xlim(SL.mapSize);
ylim(SL.mapSize);
daspect([1 1 1]);

%Setup colorbar
colorbar;
cbLimits = [-1,SL.thermalStrengthMax];
set(gca,'clim',cbLimits);

%Setup colormap color-scheme
xColor = linspace(0,SL.thermalPixels,length(SL.CMColors)) / SL.thermalPixels;
map = interp1(xColor,SL.CMColors,linspace(0,1,SL.thermalPixels)); % Creates a color gradient for the map
colormap(map);

%% Create instance of simulation
swarm = Swarm(simLaw,thermalMap);
%theta = linspace(0,2*pi,50);
%patchX = 50*cos(theta)-0;
%patchY = 50*sin(theta)-0;
% patchX = 600*cos(theta)-1000;
% patchY = 600*sin(theta)+1000;
%patchObj = patch('XData',patchX,'YData',patchY,'FaceColor','red','FaceAlpha',0.8);

%% Prepare data
steps = simLaw.totalTime/simLaw.dt;
timeAxis = simLaw.dt * (1:steps);
maxHeights = zeros(1,steps);
minHeights = zeros(1,steps);
avgHeights = zeros(1,steps);

%% Run simulation
for step = 1:steps
    c1 = clock;
    %% Set up frame
    fprintf("Frame %g/%g\n",step,steps);
        
    %% Render thermals and agents
    thermalMap.renderThermals();

    swarm.renderAgents();
    
    %% Save frame
    currFrame = getframe(simFig);
    writeVideo(video,currFrame);
    pause(0.0001);
    
    %% Step physics
    thermalMap.staticStep();

    swarm.saveAgentData();
    swarm.stepSimulation();

     % Print number of Living Agents
    Living = nnz([swarm.agents.isAlive]);
    fprintf("%g Agents, ", Living);
    
    maxHeight = simLaw.agentFloor;
    minHeight = simLaw.agentCeiling;
    averageHeight = 0;
    for i=1:simLaw.numAgents
        currentHeight = swarm.agents(i).position(3);
        maxHeight = max(maxHeight,currentHeight);
        if(currentHeight > 0)
            minHeight = min(minHeight,currentHeight);
        end
        averageHeight = averageHeight + currentHeight;
    end
    averageHeight = averageHeight / simLaw.numAgents;
    
    maxHeights(step) = maxHeight;
    minHeights(step) = minHeight;
    avgHeights(step) = averageHeight;
    
    stringTitle = sprintf("Agents Alive: %g\nMax Height: %.1f\nMin Height: %.1f\nAverage Height: %.1f",Living,maxHeight,minHeight,averageHeight);
    title(stringTitle);
    
    %% Find and print elapsed time
    c2 = clock;
    elapsedTime = c2(6)-c1(6);
    % If minute advances, elapsedTime will appear negative (1min20sec - 0min50sec = 20sec-50sec = -30sec)
    if(elapsedTime < 0) 
        elapsedTime = elapsedTime + 60;
    end
    fprintf("%g sec\n",elapsedTime);
end

close(video);