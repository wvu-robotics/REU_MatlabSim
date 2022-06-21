% Main script: loads parameter variables and runs swarm step function
%% Clear
close all
clear
clc

%% Add search paths for sim laws and agent functions
addpath("Code of Laws");
addpath("Agent Control Functions");
addpath("Find Neighborhood Functions");

%% Load simulation parameters
simLaw = AdamsLaw();

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
patchX = 50*cos(theta)-0;
patchY = 50*sin(theta)-0;
% patchX = 600*cos(theta)-1000;
% patchY = 600*sin(theta)+1000;
patchObj = patch('XData',patchX,'YData',patchY,'FaceColor','red','FaceAlpha',0.8);

%% Prepare data
steps = simLaw.totalTime/simLaw.dt;
timeAxis = simLaw.dt * (1:steps);
maxHeights = zeros(1,steps);
minHeights = zeros(1,steps);
avgHeights = zeros(1,steps);

%% Run simulation
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
    

    % Find and print elapsed time
    c2 = clock;
    elapsedTime = c2(6)-c1(6);
    % If minute advances, elapsedTime will appear negative (1min20sec - 0min50sec = 20sec-50sec = -30sec)
    if(elapsedTime < 0) 
        elapsedTime = elapsedTime + 60;
    end
    fprintf("%g sec\n",elapsedTime);
end

simData = figure();
hold on
xlim([1, timeAxis(steps)]);
ylim([simLaw.agentFloor, simLaw.agentCeiling]);
xlabel("Time (s)");
ylabel("Height (m)");
plot(timeAxis,maxHeights);
plot(timeAxis,minHeights);
plot(timeAxis,avgHeights);
legend("Max Height","Minimum Height","Average Height",'Location','southeast');
saveas(simData,picName);

close(video);