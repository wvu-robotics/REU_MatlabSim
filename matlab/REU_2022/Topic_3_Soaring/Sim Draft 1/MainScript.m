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
%SL = AdamsLaw();
SL = BigSL(1);

%% setup output folder
rootFolder = "Output Media";
dateFormat = "mm-dd-yy";
timeFormat = "HH-MM-SS";
videoPrefix = "TestVideo";
%videoPrefix = sprintf('[dt %g, T %g, x%g] ',simLaw.dt, simLaw.totalTime, simLaw.fpsMult);

% Get current date/time
date = datestr(now,dateFormat);
time = datestr(now,timeFormat);

% Create folder/video names
dateFolder = sprintf('%s/%s',rootFolder,date);

%videoSuffix = sprintf('S=%1.0E, C=%1.0E, A=%1.0E, M=%1.0E', simLaw.separation, simLaw.cohesion, simLaw.alignment, simLaw.migration);
%videoName = sprintf('%s/%s %s.avi',dateFolder,videoPrefix,videoSuffix);
videoName = sprintf('%s/%s %s.avi',dateFolder,videoPrefix,time);
picName = sprintf('%s/%s %s.png',dateFolder,videoPrefix,time);

% Create folders if they don't exist
if(~exist(rootFolder,'dir'))
    mkdir(rootFolder);
end

if(~exist(dateFolder,'dir'))
    mkdir(dateFolder);
end

%% Setup video and figure
%Create video
video = VideoWriter(videoName);
video.FrameRate = 1/SL.dt * SL.fpsMult / SL.frameSkip;
open(video);

%Create frame
simFig = figure('Visible','off');
xlim(SL.mapSize);
ylim(SL.mapSize);
daspect([1 1 1])

%Setup colorbar
colorbar;
cbLimits = [-1,SL.thermalStrengthMax];
set(gca,'clim',cbLimits);

%Setup colormap color-scheme
xColor = linspace(0,SL.thermalPixels,length(SL.CMColors)) / SL.thermalPixels;
map = interp1(xColor,SL.CMColors,linspace(0,1,SL.thermalPixels)); % Creates a color gradient for the map
colormap(map);

%% Create objects
thermalMap = ThermalMap(SL);
swarm = Swarm(SL, thermalMap);

%% Prepare data
steps = SL.totalTime/SL.dt;
timeAxis = SL.dt * (1:steps);
maxHeights = zeros(1,steps);
minHeights = zeros(1,steps);
avgHeights = zeros(1,steps);
flightScore = 0;

c1 = clock;
%% Run simulation
simFig.Visible = 'on';
for step = 1:steps
    if(mod(step,SL.frameSkip)==0)
        % Find and print elapsed time
        c2 = clock;
        elapsedTime = c2(6)-c1(6);
        c1 = c2;
        % If minute advances, elapsedTime will appear negative (1min20sec - 0min50sec = 20sec-50sec = -30sec)
        if(elapsedTime < 0) 
            elapsedTime = elapsedTime + 60;
        end
        fprintf("Frame %g/%g:  %.3f\n",step,steps,elapsedTime);
        
        % Render agents and thermals
        thermalMap.renderThermals();
        swarm.renderAgents();
        
        % Save video frame
        currFrame = getframe(simFig);
        writeVideo(video,currFrame);
    end
    
    % Step simulation
    thermalMap.staticStep();
    swarm.saveAgentData();
    swarm.stepSimulation();
    
    % Print number of Living Agents
    Living = nnz([swarm.agents.isAlive]);
    %fprintf("%g Agents, ", Living);
    flightScore = flightScore + Living * SL.dt;
    
    [maxHeight, minHeight, avgHeight, avgSpeed] = swarm.getData();
    
    maxHeights(step) = maxHeight;
    minHeights(step) = minHeight;
    avgHeights(step) = avgHeight;
    
    stringTitle = sprintf("Agents Alive: %g\nMax Height: %.1f\nMin Height: %.1f\nAverage Height: %.1f\nAverage Speed: %.1f",Living,maxHeight,minHeight,avgHeight,avgSpeed);
    title(stringTitle);   
end

simData = figure();
hold on
xlim([1, timeAxis(steps)]);
ylim([SL.agentFloor, SL.agentCeiling]);
xlabel("Time (s)");
ylabel("Height (m)");
plot(timeAxis,maxHeights);
plot(timeAxis,minHeights);
plot(timeAxis,avgHeights);
legend("Max Height","Minimum Height","Average Height",'Location','southeast');
plotTitle = sprintf("Total Flight Score: %.1f",flightScore);
saveas(simData,picName);

close(video);