%% Setup Workspace
clear
clc
close all

%% Save Data
saveFigs = true;
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

%% Find Chosen Agents, If Necessary
% Used to select agents: 9,11,27,29
if(false)
    % Time stamp = 0:29:24
    timeFrame = 14400 * ((24/60)+29)/120;
    targetXRange = [-500,1000];
    targetYRange = [-1500,-300];
    % Use annotated agent to find time
    for agentNum = 1:size(xData,1)
        validFrames = find(xData(agentNum,timeFrame) > targetXRange(1) & ...
                           xData(agentNum,timeFrame) < targetXRange(2) & ...
                           yData(agentNum,timeFrame) > targetYRange(1) & ...
                           yData(agentNum,timeFrame) < targetYRange(2));
        if(~isempty(validFrames))
            fprintf("Agent %g is valid at timestep %g.\n",agentNum,timeFrame);
        end
    end
    return
end

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
chosenAgents = [9,11,27,29];
chosenTimeSteps = (3350:3650);

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

%% Render BadDiscoveryTimeZVelCoh
fprintf("Rendering figure: BadDiscoveryTimeZVelCoh... ");
fig1 = figure('Name','BadDiscoveryTimeZVelCoh');
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
c.Ticks = [1675:50:1825];
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
c.Ticks = [-1,0,1,2,2.7];
c.TickLabels = {c.Ticks};
c.Title.String = "[m/s]";
c.Title.Units = 'normalized';
c.Title.Position = [2.8, 0.5, 0];
c.Title.FontSize = 10;
c.Title.FontName = "Arial";
c.Title.Color = [0.1,0.1,0.1];
c.Title.HorizontalAlignment = 'center';
c.Title.VerticalAlignment = 'middle';
casd = c;
% Subfigure: Top Coh
nexttile
plotPaths(xData,yData,zData,cohMag);
a = gca;
a.DataAspectRatio = [1,1,1];
view(0,90);
subTitle("(c)");
c = colorbar;
c.Ticks = [0,2,4,6,7.8];
c.TickLabels = {c.Ticks};

fig1.Position = [300,400,1000,400];
fprintf("Done!\n");

%% Render DataBadDiscovery
dataSize = size(xData,2);
lineThickness = 1;
time = time(1,:);

fprintf("Rendering figure: DataBadDiscovery... ");
fig2 = figure('Name','DataBadDiscovery');
c = colormap(jet);
tiledlayout(3,1);
nexttile
hold on
for i=1:length(chosenAgents)
    col = c(floor(i/length(chosenAgents)*256),:);
    plot(time,zData(i,:),'Color',col,'LineWidth',lineThickness);
end
markers = ['x','o','*','+'];
for i=1:length(chosenAgents)
    col = c(floor(i/length(chosenAgents)*256),:);
    markAgentData(time(1),zData(i,1),markers(i),col);
end
xlim([min(time),max(time)]);
xlabel("Time [s]");
ylabel("Vertical Height [m]");
a = gca;
a.XTick = (1675:25:1825);
a.XTickLabel = {a.XTick};
%ylim([1150, 1800]);
hold off
nexttile
hold on
for i=1:length(chosenAgents)
    col = c(floor(i/length(chosenAgents)*256),:);
    plot(time,zVelData(i,:),'Color',col,'LineWidth',lineThickness);
end
markers = ['x','o','*','+'];
for i=1:length(chosenAgents)
    col = c(floor(i/length(chosenAgents)*256),:);
    markAgentData(time(1),zVelData(i,1),markers(i),col);
end
xlim([min(time),max(time)]);
xlabel("Time [s]");
ylabel("Vertical Speed [m/s]");
a = gca;
a.XTick = (1675:25:1825);
a.XTickLabel = {a.XTick};
hold off
nexttile
hold on
for i=1:length(chosenAgents)
    col = c(floor(i/length(chosenAgents)*256),:);
    plot(time,cohMag(i,:),'Color',col,'LineWidth',lineThickness);
end
markers = ['x','o','*','+'];
for i=1:length(chosenAgents)
    col = c(floor(i/length(chosenAgents)*256),:);
    markAgentData(time(1),cohMag(i,1),markers(i),col);
end
xlim([min(time),max(time)]);
xlabel("Time [s]");
ylabel("Cohesion Magnitude");
a = gca;
a.XTick = (1675:25:1825);
a.XTickLabel = {a.XTick};
legend(["Agent 9","Agent 11","Agent 27","Agent 29"],'Location','northeast');
hold off

fprintf("Done!\n");

%% Saving Figures
if(saveFigs)
    fprintf("Saving BadDiscoveryTimeZVelCoh.png... ");
    exportgraphics(fig1,'BadDiscoveryTimeZVelCoh.png','Resolution',saveDPI);
    fprintf("Done!\n");

    fprintf("Saving DataBadDiscovery.png... ");
    exportgraphics(fig2,'DataBadDiscovery.png','Resolution',saveDPI);
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
    markers = ['x','o','*','+'];
    for i=1:numAgents
        markAgentPath(X,Y,Z,i,markers(i));
    end
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