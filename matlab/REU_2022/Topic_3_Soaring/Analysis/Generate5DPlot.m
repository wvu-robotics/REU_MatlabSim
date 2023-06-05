% This program can be used to plot many results of a megarun in a single
% graph, allowing intuitive observation of trends. The generated graph can
% be laggy when many points are plotted, so be sure to tweak
% numPointstoPlot.

%% Prepare workspace
clear;
clc;
close all;

%% Load data
data = load("../../Megaruns/Megarun_7/CombinedData_20220725165445.mat");
numPointsToPlot = 5000;

%% Prepare data for plotting
len = length(data.flightTime);
normedFT = data.flightTime/max(data.flightTime,[],"all");
color = hsv2rgb(normedFT',ones(len,1),ones(len,1));
normedSurvivingPercent = interp1([0,100],[2,10],data.survivingPercent);
simsToPlot = randperm(len,numPointsToPlot);

%% Plot data
fprintf("Plotting points... ");
hold on
for i = 1:length(simsToPlot)
    simToPlot = simsToPlot(i);
    plot3(data.heightScore(simToPlot),data.explorationPercent(simToPlot),data.thermalUseScore(simToPlot),'.','Color',color(simToPlot,1,:),'MarkerSize',normedSurvivingPercent(simToPlot));
end

xlabel("Height Score");
ylabel("Exploration Percent");
zlabel("Thermal Use Score");
legend("Color is Flight Time, Size is Surviving Percent")

ax = gca;
ax.Color = [0.2 0.2 0.2];
view(3)

hold off
fprintf("Done!\n");