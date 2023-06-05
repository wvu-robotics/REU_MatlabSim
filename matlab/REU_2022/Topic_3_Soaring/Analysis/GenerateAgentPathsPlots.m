% This program can be used to provide a quick understanding of the agent
% paths taken during the duration of the simulation, including agent death.
%% Prepare workspace
clear;
clc;
close all;

%% Load data
data = load("../../Output_Media/06-05-23/SimBatch_20230605155220/MatFiles/1_OutputData.mat");
stepSkip = 1; % Program will plot every stepSkip frame(s)

%% Prepare data
fprintf("Preparing data... ");
dataSize = size(data.xData,2);
numAgents = data.SL.numAgents;
color = (1:dataSize)/dataSize;
timeTicks = 1:(data.SL.totalTime/data.SL.dt);
time = timeTicks * data.SL.dt;

% Cut data
cutData = mod(1:dataSize,stepSkip)~=0;

data.bankAngleData(:,cutData) = [];
data.headingData(:,cutData) = [];
data.fVelData(:,cutData) = [];
data.heightData(:,cutData) = [];
data.tVelData(:,cutData) = [];
data.xData(:,cutData) = [];
data.yData(:,cutData) = [];
data.zData(:,cutData) = [];
data.zVelData(:,cutData) = [];
color(:,cutData) = [];
time(:,cutData) = [];
fprintf("Done!\n");

%% Parse agent death data
fprintf("Parsing agent death data... ");
tickOfDeath = NaN(1,numAgents);
isDead = data.zData<=1;
for agent = 1:numAgents
    tempTOD = find(isDead(agent,:), 1, "first");
    if(~isempty(tempTOD))
        tickOfDeath(agent) = tempTOD;
    end
end
deadxData = data.xData;
deadxData(~isDead) = NaN;
deadyData = data.yData;
deadyData(~isDead) = NaN;
deadzData = data.zData;
deadzData(~isDead) = NaN;
lastX = diag(data.xData(~isnan(tickOfDeath),tickOfDeath(~isnan(tickOfDeath))));
lastY = diag(data.yData(~isnan(tickOfDeath),tickOfDeath(~isnan(tickOfDeath))));
lastZ = diag(data.zData(~isnan(tickOfDeath),tickOfDeath(~isnan(tickOfDeath))));
fprintf("Done!\n");

%% Generate plots
fprintf("Generating path plot... ");
pathFig3D = figure("Name","Agent Path Data","NumberTitle","off");
colormap(hsv)
axis square
hold on
patch([data.xData nan(numAgents,1)]',[data.yData nan(numAgents,1)]',[data.zData nan(numAgents,1)]',repmat([color nan(1,1)]',1,numAgents),'FaceColor','none','EdgeColor','interp')
view(3);
plot3(lastX,lastY,lastZ,'rx')
xlabel("X [m]");
ylabel("Y [m]");
zlabel("Z [m]");
hold off
fprintf("Done!\n");

fprintf("Generating max/min/avg height plot... ");
heightFig = figure("Name","Max/Min/Avg Height Data","NumberTitle","off");
hold on
plot(time, data.heightData(1,:)); % Max height
plot(time, data.heightData(2,:)); % Min height
plot(time, data.heightData(3,:)); % Avg height
legend(["Max Height","Min Height","Avg Height"]);
xlabel("Time [s]");
ylabel("Height [m]");
hold off
fprintf("Done!\n");

fprintf("Generating Z-velocity plot... ");
zVelFig = figure("Name","Z-Velocity Data","NumberTitle","off");
plot(data.zVelData(1:40,:)')
fprintf("Done!\n");

fprintf("Generating Z-position plot... ");
heightPlot = figure("Name","Z-Position Data","NumberTitle","off");
hold on
for agent = 1:numAgents
    plot(time,data.zData(agent,:));
    plot(time(~isnan(tickOfDeath)),lastZ,'rx');
end
xlabel("Time [s]");
ylabel("Z-Position [m]");
hold off
fprintf("Done!\n");

cherrypick = 1;
fprintf("Generating path plot of agent %g... ", cherrypick);
figName = sprintf("Agent Path Data (Agent %g)",cherrypick);
singlePathFig3D = figure("Name",figName,"NumberTitle","off");
colormap(hsv)
patch([data.xData(cherrypick,:) nan]',[data.yData(cherrypick,:) nan]',[data.zData(cherrypick,:) nan]',[color nan]','FaceColor','none','EdgeColor','interp')
view(3);
xlabel("X [m]");
ylabel("Y [m]");
zlabel("Z [m]");
fprintf("Done!\n");