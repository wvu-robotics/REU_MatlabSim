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
color_time = repmat(color_time,numAgents,1);
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
%color_cohMag = (cohMag - minCohMag)/(maxCohMag - minCohMag);
%color_cohMag(:,1:2) = color_bounds;
%color_cohMag(color_cohMag > 0.2) = 0.2;
color_cohMag = cohMag;


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

%% Choose Data
chosenAgents = [19,30];
chosenTimeSteps = (1:300);

xData = xData(chosenAgents,chosenTimeSteps);
yData = yData(chosenAgents,chosenTimeSteps);
zData = zData(chosenAgents,chosenTimeSteps);
time = repmat(time(chosenTimeSteps),length(chosenAgents),1);
zVelData = zVelData(chosenAgents,chosenTimeSteps);
cohMag = cohMag(chosenAgents,chosenTimeSteps);
%cohMag(cohMag > 20) = 20;

color_time = color_time(chosenAgents,chosenTimeSteps);
color_zVel = color_zVel(chosenAgents,chosenTimeSteps);
color_cohMag = color_cohMag(chosenAgents,chosenTimeSteps);

%% Render PathsAgent19vs30Time
fprintf("Rendering figure: PathsAgent19vs30Time... ");
fig1 = figure('Name','PathsAgent19vs30Time');
colormap jet
tiledlayout(1,3);
% Subfigure: Top Corner
nexttile
plotPaths(xData,yData,zData,time);
a = gca;
a.DataAspectRatio = [1,1,1];
view(30,22);
subTitle("(a)");
% Subfigure: Side
nexttile
plotPaths(xData,yData,zData,time);
a = gca;
a.DataAspectRatio = [1,1,1];
view(0,0);
subTitle("(b)");
% Subfigure: Top
nexttile
plotPaths(xData,yData,zData,time);
a = gca;
a.DataAspectRatio = [1,1,1];
view(0,90);
subTitle("(c)");
c = colorbar;
c.Ticks = [1,50,100,150];
c.TickLabels = {c.Ticks};
c.Title.String = "[s]";
c.Title.Units = 'normalized';
c.Title.Position = [3, 0.5, 0];
c.Title.FontSize = 10;
c.Title.FontName = "Arial";
c.Title.Color = [0.1,0.1,0.1];
c.Title.HorizontalAlignment = 'center';
c.Title.VerticalAlignment = 'middle';

fig1.Position = [300,400,1000,400];
fprintf("Done!\n");

%% Render PathsAgent19vs30TimeZVelCoh
fprintf("Rendering figure: PathsAgent19vs30TimeZVelCoh... ");
fig2 = figure('Name','PathsAgent19vs30TimeZVelCoh');
colormap jet
tiledlayout(1,3);
% Subfigure: Top Time
nexttile
plotPaths(xData,yData,zData,time);
a = gca;
a.DataAspectRatio = [1,1,1];
view(0,90);
subTitle("(a)");
c = colorbar;
c.Ticks = [1, 50:50:300];
c.TickLabels = {c.Ticks};
c.Title.String = "[s]";
c.Title.Units = 'normalized';
c.Title.Position = [3, 0.5, 0];
c.Title.FontSize = 10;
c.Title.FontName = "Arial";
c.Title.Color = [0.1,0.1,0.1];
c.Title.HorizontalAlignment = 'center';
c.Title.VerticalAlignment = 'middle';
% Subfigure: Top ZVel
nexttile
plotPaths(xData,yData,zData,zVelData);
a = gca;
a.DataAspectRatio = [1,1,1];
view(0,90);
subTitle("(b)");
c = colorbar;
c.Title.String = "[m/s]";
c.Title.Units = 'normalized';
c.Title.Position = [2.8, 0.5, 0];
c.Title.FontSize = 10;
c.Title.FontName = "Arial";
c.Title.Color = [0.1,0.1,0.1];
c.Title.HorizontalAlignment = 'center';
c.Title.VerticalAlignment = 'middle';
% Subfigure: Top Coh
nexttile
plotPaths(xData,yData,zData,cohMag);
a = gca;
a.DataAspectRatio = [1,1,1];
view(0,90);
subTitle("(c)");
colorbar;

fig2.Position = [300,400,1000,400];
fprintf("Done!\n");

%% Render DataAgent19vs30
lineThickness = 1;
time = time(1,:);

fprintf("Rendering figure: DataAgent19vs30... ");
fig3 = figure('Name','DataAgent19vs30');
tiledlayout(3,1);
nexttile
hold on
plot(time,zData(1,:),'Color',[0,0,1],'LineWidth',lineThickness);
plot(time,zData(2,:),'Color',[1,0,0],'LineWidth',lineThickness);
markAgentData(1,zData(1,1),'x',[0,0,1]);
markAgentData(1,zData(2,1),'o',[1,0,0]);
xlabel("Time [s]");
ylabel("Vertical Height [m]");
%ylim([1150, 1800]);
hold off
nexttile
hold on
plot(time,zVelData(1,:),'Color',[0,0,1],'LineWidth',lineThickness);
plot(time,zVelData(2,:),'Color',[1,0,0],'LineWidth',lineThickness);
markAgentData(1,zVelData(1,1),'x',[0,0,1]);
markAgentData(1,zVelData(2,1),'o',[1,0,0]);
xlabel("Time [s]");
ylabel("Vertical Speed [m/s]");
hold off
nexttile
hold on
plot(time,cohMag(1,:),'Color',[0,0,1],'LineWidth',lineThickness);
plot(time,cohMag(2,:),'Color',[1,0,0],'LineWidth',lineThickness);
markAgentData(1,cohMag(1,1),'x',[0,0,1]);
markAgentData(1,cohMag(2,1),'o',[1,0,0]);
xlabel("Time [s]");
ylabel("Cohesion Magnitude");
legend(["Agent 19","Agent 30"],'Location','northwest');
hold off

fprintf("Done!\n");

%% Saving Figures
if(saveFigs)
    fprintf("Saving PathsAgent19vs30Time.png... ");
    exportgraphics(fig1,'PathsAgent19vs30Time.png','Resolution',saveDPI);
    fprintf("Done!\n");

    fprintf("Saving PathsAgent19vs30TimeZVelCoh.png... ");
    exportgraphics(fig2,'PathsAgent19vs30TimeZVelCoh.png','Resolution',saveDPI);
    fprintf("Done!\n");

    fprintf("Saving DataAgent19vs30.png... ");
    exportgraphics(fig3,'DataAgent19vs30.png','Resolution',saveDPI);
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
    patch(X',Y',Z',C','FaceColor','none','EdgeColor','interp','EdgeAlpha',1,'LineWidth',1);
    axis square
    xlabel("X [m]");
    ylabel("Y [m]");
    zlabel("Z [m]");
    markAgentPath(X,Y,Z,1,'x');
    markAgentPath(X,Y,Z,2,'o');
end

function subTitle(titleName)
    title(titleName,'Units','normalized','Position',[0.5,-0.35,0],'HorizontalAlignment','center');
end

function markAgentPath(X,Y,Z,agentNum,mark)
    hold on
    scatter3(X(agentNum,1),Y(agentNum,1),Z(agentNum,1),mark,'black');
    hold off
end

function markAgentData(X,Y,mark,color)
    hold on
    scatter(X,Y,[],color,mark);
    hold off
end