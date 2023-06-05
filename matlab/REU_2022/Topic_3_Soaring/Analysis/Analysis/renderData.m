clear
clc
close all
disp("Loading Workspace")
load Output_Media\07-25-22\SimBatch_20220725133047\MatFiles\1_OutputData.mat

dataSize = size(xData,2);
numAgents = size(xData,1);
color = (1:dataSize)/dataSize;
stepSkip = 1;
cutData = mod(1:dataSize,stepSkip)~=0;
%% trim fine data
bankAngleData(:,cutData) = [];
headingData(:,cutData) = [];
fVelData(:,cutData) = [];
heightData(:,cutData) = [];
tVelData(:,cutData) = [];
xData(:,cutData) = [];
yData(:,cutData) = [];
zData(:,cutData) = [];
zVelData(:,cutData) = [];
color(:,cutData) = [];
%% do the rest


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
pathFig3D = figure;
colormap(hsv)
disp("Working on 3D Plot...")
axis square
patch([xData nan(numAgents,1)]',[yData nan(numAgents,1)]',[zData nan(numAgents,1)]',repmat([color nan(1,1)]',1,numAgents),'FaceColor','none','EdgeColor','interp')
hold on
view(3);

plot3(lastX,lastY,lastZ,'rx')

heightFig = figure;
disp("Working on Height Plot...")
plot(heightData')
zVelFig = figure;
disp("Working on Ascension Plot...")
plot(zVelData(1:40,:)')
heightPlot = figure;
disp("Working on regular Z Plot...")
plot(zData(1:40,:)')
hold on
plot(timeOfDeath(~isnan(timeOfDeath)),lastZ,'rx')


cherrypick = 1;
singlePathFig3D = figure;
colormap(hsv)
patch([xData(cherrypick,:) nan]',[yData(cherrypick,:) nan]',[zData(cherrypick,:) nan]',[color nan]','FaceColor','none','EdgeColor','interp')
view(3);

