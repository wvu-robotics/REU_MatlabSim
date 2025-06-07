%% Setup Workspace
clear
clc
close all

%{
m = find(data.survivingPercent == 100);
varNames = ["cohesion","separation","alignment","cohPower","migration","numThermals","numAgents","rngSeed"];
for i=1:length(m)
    for j=1:length(varNames)
        p = sprintf("%s=%g, ", varNames(j), data.(varNames(j))(m(i)));
        fprintf("%20s",p);
    end
    fprintf("survivingPercent=%g\n",data.survivingPercent(m(i)));
end

clc;
close all;
figure
view(3)
xData = data.(xName);
yData = data.(yName);
zData = data.(zName);
m = unique(data.migration);
na = unique(data.numAgents);
mask1 = data.migration == m(1) & data.numAgents == na(1);
mask2 = data.migration == m(1) & data.numAgents == na(2);
mask3 = data.migration == m(2) & data.numAgents == na(1);
mask4 = data.migration == m(2) & data.numAgents == na(2);
mask5 = data.migration == m(3) & data.numAgents == na(1);
mask6 = data.migration == m(3) & data.numAgents == na(2);
hold on
scatter3(xData(mask1), yData(mask1), zData(mask1), 3.5, c_red, "filled");
scatter3(xData(mask2), yData(mask2), zData(mask2), 3.5, c_orange, "filled");
scatter3(xData(mask3), yData(mask3), zData(mask3), 3.5, c_yellow, "filled");
scatter3(xData(mask4), yData(mask4), zData(mask4), 3.5, c_green, "filled");
scatter3(xData(mask5), yData(mask5), zData(mask5), 3.5, c_lblue, "filled");
scatter3(xData(mask6), yData(mask6), zData(mask6), 3.5, c_blue, "filled");
hold off
%}

%% Save Data
saveFigs = false;
saveDPI = 1500; %600

%% Load Variables
fprintf("Loading Variables... ");
data = load("..\..\Data\Megarun7Unified.mat");
fprintf("Done!\n");

xName = "thermalUseScore";
yName = "explorationPercent";
zName = "survivingPercent";

%% Create figures for each variable
varNames = ["cohesion","separation","alignment","cohPower","migration","numThermals","numAgents","rngSeed"];

c_red    = [  1,   0, 0];
c_orange = [  1, 0.7, 0];
c_yellow = [0.9,   1, 0];
c_green  = [  0,   1, 0];
c_lblue  = [  0, 0.9, 1];
c_blue   = [  0,   0, 1];
colors = {c_red;
          [c_red; c_blue];
          [c_red; c_green; c_blue];
          [c_red; c_orange; c_green; c_blue];
          [c_red; c_orange; c_green; c_lblue; c_blue]};
screenUnit = 400; % 600      
      
%% Generate All Metric Data
allFigName = "AllMetricData";
fig_all = figure("Name", allFigName, "NumberTitle", "off");
tiledlayout(1,3);
% Isometric
nexttile;
scatterPlot_left = plotMetrics(data, xName, yName, zName, allFigName, c_blue);
view(-37.5, 30);
scatterPlot_left.SizeData = 3.5;
% XZ
nexttile;
scatterPlot_middle = plotMetrics(data, xName, yName, zName, allFigName, c_blue);
view(0,0);
scatterPlot_middle.SizeData = 10;
% YZ
nexttile;
scatterPlot_right = plotMetrics(data, xName, yName, zName, allFigName, c_blue);
view(-90,0);
scatterPlot_right.SizeData = 10;

fig_all.Units = "normalized";
fig_all.Position(1:2) = [0.2, 0.2];
fig_all.Units = "pixels";
fig_all.Position(3:4) = [3.3*screenUnit,screenUnit];

%{
f = figure(2);
t = f.Children;
ax = t.Children(1);

%}

%% Generate Comparison of High Priority Variables: migration, numAgents
highFigName = "HighPriorityVariableComparison";
fig_high = figure("Name", highFigName, "NumberTitle", "off");
tiledlayout(1, 15);
nexttile(1, [1,7]);
plotVariableComparison(data, "migration", colors);
nexttile(9, [1,7]);
plotVariableComparison(data, "numAgents", colors);
fig_high.Units = "normalized";
fig_high.Position(1:2) = [0.2, 0.2];
fig_high.Units = "pixels";
fig_high.Position(3:4) = [2.3*screenUnit,0.95*screenUnit];

%% Generate Comparison of Medium Priority Variables: cohesion, separation, numThermals
mediumFigName = "MediumPriorityVariableComparison";
fig_medium = figure("Name", mediumFigName, "NumberTitle", "off");
tiledlayout(1,3);
nexttile;
plotVariableComparison(data, "cohesion", colors);
nexttile;
plotVariableComparison(data, "separation", colors);
nexttile;
plotVariableComparison(data, "numThermals", colors);
fig_medium.Units = "normalized";
fig_medium.Position(1:2) = [0.1, 0.1];
fig_medium.Units = "pixels";
fig_medium.Position(3:4) = [3.3*screenUnit,screenUnit];

%% Generate Comparison of Low Priority Variables: alignment, cohPower, rngSeed
lowFigName = "LowPriorityVariableComparison";
fig_low = figure("Name", lowFigName, "NumberTitle", "off");
tiledlayout(1, 3);
nexttile;
plotVariableComparison(data, "alignment", colors);
nexttile;
plotVariableComparison(data, "cohPower", colors);
nexttile;
plotVariableComparison(data, "rngSeed", colors);
fig_low.Units = "normalized";
fig_low.Position(1:2) = [0.2, 0.2];
fig_low.Units = "pixels";
fig_low.Position(3:4) = [3.3*screenUnit,screenUnit];

%% Saving Figures
if(saveFigs)
    folderPrefix = "FormattedSurface";
    
    allFigSaveName = sprintf("%s\\%s.png",folderPrefix,allFigName);
    fprintf("Saving %s... ", allFigSaveName);
    exportgraphics(fig_all,allFigSaveName,'Resolution',saveDPI);
    fprintf("Done!\n");
    
    highFigSaveName = sprintf("%s\\%s.png",folderPrefix,highFigName);
    fprintf("Saving %s... ", highFigSaveName);
    exportgraphics(fig_high,highFigSaveName,'Resolution',saveDPI);
    fprintf("Done!\n");
    
    mediumFigSaveName = sprintf("%s\\%s.png",folderPrefix,mediumFigName);
    fprintf("Saving %s... ", mediumFigSaveName);
    exportgraphics(fig_medium,mediumFigSaveName,'Resolution',saveDPI);
    fprintf("Done!\n");
    
    lowFigSaveName = sprintf("%s\\%s.png",folderPrefix,lowFigName);
    fprintf("Saving %s... ", lowFigSaveName);
    exportgraphics(fig_low,lowFigSaveName,'Resolution',saveDPI);
    fprintf("Done!\n");
end

%% Helper Functions
function scatterPlot = plotMetrics(data, xName, yName, zName, figName, color)
    plotAlpha = 1;
    plotCircleSize = 3.5;
    fontSize = 17;
    fprintf("Plotting %s... ", figName);
    
    xData = data.(xName);
    yData = data.(yName);
    zData = data.(zName);
    
    scatterPlot = scatter3(xData, yData, zData, plotCircleSize, color, "filled", "MarkerFaceAlpha", plotAlpha);
    xlabel(xName);
    ylabel(yName);
    zlabel(zName);
    
    a = gca;
    a.FontSize = fontSize;
    fprintf("Done!\n");
end

function plotVariableComparison(data, varName, colors)
    xName = "thermalUseScore";
    yName = "explorationPercent";
    zName = "survivingPercent";

    fprintf("Plotting comparison for %s... ", varName);
    titleFontSize = 20*2/3; % 20
    fontSize = 15*2/3;
    plotAlpha = 1;
    plotCircleSize = 1.5; % 3.5
    
    xData = data.(xName);
    yData = data.(yName);
    zData = data.(zName);
    
    varValues = unique(data.(varName));
    numValues = length(varValues);
    
    view(3);
    hold on;
    
    plotTitle = sprintf("\\fontsize{%g}%s = ",titleFontSize,varName);
    for j=1:numValues
        varValue = varValues(j);
        varMask = data.(varName) == varValue;
        colorMap = colors{numValues, 1};
        currColor = colorMap(j,:);
        plotTitle = plotTitle + sprintf("\\color[rgb]{%g,%g,%g}%g",currColor(1),currColor(2),currColor(3),varValue);
        if(j<numValues)
            plotTitle = plotTitle + ", ";
        end
        
        %plot3(xData(varMask), yData(varMask), zData(varMask),'.',"Color",currColor);
        scatter3(xData(varMask), yData(varMask), zData(varMask), plotCircleSize, currColor, "filled", "MarkerFaceAlpha", plotAlpha);
    end
    
    title(plotTitle,'interpreter','tex');
    xlab = xlabel("Thermal Use Score");
    xlab.Units = "normalized";
    xlab.Position = [0.6, 0, 0];
    ylab = ylabel("Exploration Percent");
    ylab.Units = "normalized";
    ylab.Position = [0.25, 0, 0];
    zlabel("Surviving Percent");
    a = gca;
    a.FontSize = fontSize;
    hold off;
    fprintf("Done!\n");
end