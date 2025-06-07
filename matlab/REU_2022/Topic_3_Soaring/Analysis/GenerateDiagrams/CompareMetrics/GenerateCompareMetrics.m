%% Setup Workspace
clear
clc
close all

%% Save Data
saveFigs = false;
saveDPI = 200;

%% Load Variables
fprintf("Loading Variables... ");
data = load("..\..\Data\Megarun7Unified.mat");
fprintf("Done!\n");

xName = "thermalUseScore";
yName = "explorationPercent";
zName = "survivingPercent";

%% Create comparison figure 
if(false)
    figure("Name","Compare Metrics 3-Plot","NumberTitle","off");
    plot3(data.(xName),data.(yName),data.(zName),'r.');
    xlabel(xName);
    ylabel(yName);
    zlabel(zName);
end

%% Create figures for each variable
varNames = ["cohesion","separation","alignment","cohPower","migration","numThermals","numAgents","rngSeed"];
%varNames = ["cohesion"];
sameFig = true;
titleFontSize = 20;
colors = colormap("jet");


close all
xData = data.(xName);
yData = data.(yName);
zData = data.(zName);
if(sameFig)
   fig = figure("Name","Combined Comparison Figure","NumberTitle","off");
   tiledlayout(2,4);
   fprintf("Rendering Combined Comparison Figure... ");
else
   figs = cell(length(varNames),1);
end
for i=1:length(varNames)
    varName = varNames(i);
    
    varValues = unique(data.(varName));
    numValues = length(varValues);
    
    if(sameFig)
        nexttile;
    else
        figName = sprintf("%s",varName);
        fig = figure("Name",figName,"NumberTitle","off");
        fprintf("Rendering %s Comparison Figure... ",figName);
    end
    view(3);
    hold on;
    
    figTitle = sprintf("\\fontsize{%g}%s = ",titleFontSize,varName);
    for j=1:numValues
        varValue = varValues(j);
        varMask = data.(varName) == varValue;
        colorIndex = floor((256-1)/(numValues+1)*j) + 1;
        currColor = colors(colorIndex,:);
        figTitle = figTitle + sprintf("\\color[rgb]{%g,%g,%g}%g",currColor(1),currColor(2),currColor(3),varValue);
        if(j<numValues)
            figTitle = figTitle + ", ";
        end
        
        plot3(xData(varMask), yData(varMask), zData(varMask),'.',"Color",currColor); 
    end
    
    title(figTitle,'interpreter','tex');
    xlabel(xName);
    ylabel(yName);
    zlabel(zName);
    hold off;
    
    if(~sameFig)
        fig.Position = [50+200*i,800-50*i,600,500];
        figs{i,1} = fig;
        fprintf("Done!\n");
    end
end
if(sameFig)
    fig.Position = [100, 100, 2400, 1200];
    fprintf("Done!\n");
end

%% Saving Figures
if(saveFigs)
    if(sameFig)
        fprintf("Saving Combined Comparison Figure.png... ");
        exportgraphics(fig,'CombinedComparisonFigure.png','Resolution',saveDPI);
        fprintf("Done!\n");
    else
        for i=1:length(varNames)
            figSaveName = sprintf("%sComparison.png",varNames(i));
            fprintf("Saving %s... ", figSaveName);
            exportgraphics(figs{i,1},figSaveName,'Resolution',saveDPI);
            fprintf("Done!\n");
        end
    end
end