%% Setup Workspace
clear
clc
close all

%% Save Data
saveFigs = false;
saveDPI = 1200;

%% Load Variables
fprintf("Loading Variables... ");
load("..\Data\SimBatch_40SurvivorsVerboseGreat\MatFiles\1_OutputData.mat");
fprintf("Done!\n");

dataSize = size(xData,2);
numAgents = size(xData,1);

%% Setup Data
% Trim Data
stepSkip = 1;
cutData = mod(1:dataSize,stepSkip)~=0;
%174 = turn back
%cutData(1,300:end) = true;

bankAngleData(:,cutData) = [];
headingData(:,cutData) = [];
fVelData(:,cutData) = [];
heightData(:,cutData) = [];
tVelData(:,cutData) = [];
xData(:,cutData) = [];
yData(:,cutData) = [];
zData(:,cutData) = [];
zVelData(:,cutData) = [];
cohMag(:,cutData) = [];
time = (1:dataSize) * SL.dt;

%% Setup Colors
color_bounds = [zeros(numAgents,1),ones(numAgents,1)];

%color_time = (1:dataSize)/dataSize;
color_time = time;
color_time(:,cutData) = [];
%color_time(:,1:2) = [0,1];

minZVel = min(min(zVelData));
maxZVel = max(max(zVelData));
%color_zVel = (zVelData-minZVel)/(maxZVel-minZVel);
color_zVel = zVelData;
%color_zVel(:,cutData) = [];
%color_zVel(:,1:2) = color_bounds;

minCohMag = min(min(cohMag));
maxCohMag = max(max(cohMag));
color_cohMag = (cohMag - minCohMag)/(maxCohMag - minCohMag);
color_cohMag(:,1:2) = color_bounds;
color_cohMag(color_cohMag > 0.2) = 0.2;


time_POI = [55,163,207,258];
%19 is first, 30 is second
%chosenAgents = [7,13,19,21,28,30,36]; % Chosen agents
chosenAgents = [19,30]; % Chosen agents
color_chosen = zeros(numAgents,dataSize);
color_chosen(chosenAgents,:) = 0.5;
color_chosen(:,cutData) = [];
color_chosen(:,1:2) = color_bounds;

%% Setup Death Data?
timeOfDeath = NaN(1,numAgents);
isDead = zData<=1;
for Agent = 1:numAgents
    tOD = find(isDead(Agent,:), 1 );
    if ~isempty(tOD)
        timeOfDeath(Agent) = tOD;
    end
end
[deadRow,deadColumn] = find(isDead);
deadxData = xData;
deadxData(~isDead) = NaN;
deadyData = yData;
deadyData(~isDead) = NaN;
deadzData = zData;
deadzData(~isDead) = NaN;
lastX = diag(xData(~isnan(timeOfDeath),timeOfDeath(~isnan(timeOfDeath))));
lastY = diag(yData(~isnan(timeOfDeath),timeOfDeath(~isnan(timeOfDeath))));
lastZ = diag(zData(~isnan(timeOfDeath),timeOfDeath(~isnan(timeOfDeath))));

%% Render figure AllPathsTime
%{
f = fig1;
t = f.Children;
for i=2:4
    a = t.Children(i).Position
end
for i=2:4
    t.Children(i).Title.Units = "pixels";
    t.Children(i).Title.Position
end

%}
titleOffset = -0.2;
viewingAngle = -45;
fprintf("Rendering figure: AllPathsTime... ");
fig1 = figure('Name','AllPathsTime');
colormap jet
tiledlayout(1,3);
% Subfigure: Top Corner
nexttile
plotPaths(xData,yData,zData,color_time);
view(viewingAngle,22.5);
title("(a)",'Units','normalized','Position',[0.5,-0.3,0],'HorizontalAlignment','center');
% Subfigure: Side Corner
nexttile
plotPaths(xData,yData,zData,color_time);
view(viewingAngle,0);
title("(b)",'Units','normalized','Position',[0.5,-0.65,0],'HorizontalAlignment','center');
% Subfigure: Top
nexttile
plotPaths(xData,yData,zData,color_time);
view(viewingAngle,90);
title("(c)",'Units','normalized','Position',[0.5,-0.3,0],'HorizontalAlignment','center');
fig1.Position = [300,400,1000,400];
a1 = gca;
a1.DataAspectRatio = [1,1,1];
c = colorbar;
c.Units = "normalized";
c.Position(1) = 0.93;
c.Ticks = [1, 1800:1800:7200];
c.TickLabels = {c.Ticks};
c.Title.String = "[s]";
c.Title.Units = 'normalized';
c.Title.Position = [3.6, 0.5, 0];
c.Title.FontSize = 10;
c.Title.FontName = "Arial";
c.Title.Color = [0.1,0.1,0.1];
c.Title.HorizontalAlignment = 'center';
c.Title.VerticalAlignment = 'middle';
% Fix axis title alignment
lowestPixPos = 1000000000;
for i=1:3
   nexttile(i);
   ax = gca;
   ax.Units = "pixels";
   pixPos = ax.Position(2);
   lowestPixPos = min(lowestPixPos, pixPos);
end
for i=1:3
   nexttile(i);
   ax = gca;
   ax.Units = "pixels";
   pixPos = ax.Position(2);
   pixOffset = -45;
   ax.Title.Units = "pixels";
   newPixPos = (lowestPixPos-pixPos) + pixOffset;
   ax.Title.Position(2) = newPixPos;
   %fprintf("Setting title %g to position %g.\n", i, newPixPos);
end

fprintf("Done!\n");

%% Render figure AllPathsZVel
fprintf("Rendering figure: AllPathsZVel... ");
fig2 = figure('Name','AllPathsZVel');
colormap jet
tiledlayout(1,3);
% Subfigure: Top Corner
nexttile
plotPaths(xData,yData,zData,color_zVel);
view(viewingAngle,22.5);
title("(a)",'Units','normalized','Position',[0.5,-0.3,0],'HorizontalAlignment','center');
% Subfigure: Side Corner
nexttile
plotPaths(xData,yData,zData,color_zVel);
view(viewingAngle,0);
title("(b)",'Units','normalized','Position',[0.5,-0.3,0],'HorizontalAlignment','center');
% Subfigure: Top
nexttile
plotPaths(xData,yData,zData,color_zVel);
view(viewingAngle,90);
title("(c)",'Units','normalized','Position',[0.5,-0.3,0],'HorizontalAlignment','center');
fig2.Position = [300,400,1000,400];
a = gca;
a.DataAspectRatio = [1,1,1];
c = colorbar;
c.Units = "normalized";
c.Position(1) = 0.93;
c.Ticks = [-1.9,c.Ticks,9.5];
c.TickLabels = {c.Ticks};
c.Title.String = "[m/s]";
c.Title.Units = 'normalized';
c.Title.Position = [3, 0.5, 0];
c.Title.FontSize = 10;
c.Title.FontName = "Arial";
c.Title.Color = [0.1,0.1,0.1];
c.Title.HorizontalAlignment = 'center';
c.Title.VerticalAlignment = 'middle';
% Fix axis title alignment
lowestPixPos = 1000000000;
for i=1:3
   nexttile(i);
   ax = gca;
   ax.Units = "pixels";
   pixPos = ax.Position(2);
   lowestPixPos = min(lowestPixPos, pixPos);
end
for i=1:3
   nexttile(i);
   ax = gca;
   ax.Units = "pixels";
   pixPos = ax.Position(2);
   pixOffset = -45;
   ax.Title.Units = "pixels";
   newPixPos = (lowestPixPos-pixPos) + pixOffset;
   ax.Title.Position(2) = newPixPos;
   %fprintf("Setting title %g to position %g.\n", i, newPixPos);
end
fprintf("Done!\n");

%% Saving Figures
if(saveFigs)
    figName = "AllPathsTime.eps";
    fprintf("Saving %s... ",figName);
    exportgraphics(fig1,figName,'Resolution',saveDPI);
    fprintf("Done!\n");
    figName = "AllPathsTime.png";
    fprintf("Saving %s... ",figName);
    exportgraphics(fig1,figName,'Resolution',saveDPI);
    fprintf("Done!\n");

    figName = "AllPathsZVel.eps";
    fprintf("Saving %s... ",figName);
    exportgraphics(fig2,figName,'Resolution',saveDPI);
    fprintf("Done!\n");
    figName = "AllPathsZVel.png";
    fprintf("Saving %s... ",figName);
    exportgraphics(fig2,figName,'Resolution',saveDPI);
    fprintf("Done!\n");
end


%% Helper Functions
function plotPaths(X,Y,Z,C)
    numAgents = size(X,1);
    if(size(C,1) ~= numAgents)
        C = repmat(C,numAgents,1);
    end
    X = [X,nan(numAgents)];
    Y = [Y,nan(numAgents)];
    Z = [Z,nan(numAgents)];
    C = [C,nan(numAgents)];
    patch(X',Y',Z',C','FaceColor','none','EdgeColor','interp','EdgeAlpha',0.2);
    axis square
    xlabel("X [m]");
    ylabel("Y [m]");
    zlabel("Z [m]");
    %c = colorbar
    %p = c.Position
    %c.Position = [p(1)+0.025, p(2:4)];
end