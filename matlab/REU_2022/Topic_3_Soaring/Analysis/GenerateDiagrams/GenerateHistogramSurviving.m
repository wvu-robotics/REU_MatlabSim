%% Prepare workspace
clc;
clear;
close all;

saveFigs = false;
saveDPI = 900; %480 
fontSize = 15;

%% Load variables
fprintf("Loading in data... ");
data = load("..\Data\Megarun7Unified.mat");
fprintf("Done!\n");

%% Generate histogram
fig = figure("Name","Histogram Surviving Percent","NumberTitle","off");
fig.Units = "pixels";
fig.Position = [0,0,1400*0.8,800*0.8];

binEdges = 0:5:100;
histogram(data.survivingPercent,'BinEdges',binEdges);
xlabel("% of Surviving Agents");
ylabel("Number of Simulations");

ax = gca;
ax.FontSize = fontSize;

%% Save figure
if(saveFigs)
    figSaveName = sprintf("Unified_Surviving.png");
    fprintf("Saving figure: %s... ",figSaveName);
    exportgraphics(fig,figSaveName,'Resolution',saveDPI);
    fprintf("Done!\n");
end