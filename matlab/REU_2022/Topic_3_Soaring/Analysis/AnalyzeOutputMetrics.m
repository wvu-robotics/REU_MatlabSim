clc; clear; close all
data = load("Megarun Data\Megarun7Unified.mat");

figure;
plot3(data.surviving,data.thermalUseScore,data.explorationPercent,'.');
title("Surviving vs ThermalUseScore vs ExplorationPercent");
xlabel("Surviving");
ylabel("ThermalUseScore");
zlabel("ExplorationPercent");

%% Exploration Percent
figure;
histogram(data.explorationPercent,'BinEdges',(0:1:100));
title("ExplorationPercent");
xlabel("ExplorationPercent [%]");
ylabel("Number of Sims");

%% Thermal Use Score
figure;
maxScore = max(data.thermalUseScore);
histogram(data.thermalUseScore,'BinEdges',(0:maxScore/100:maxScore));
title("ThermalUseScore");
xlabel("ThermalUseScore");
ylabel("Number of Sims");

%% Num of Surviving Agents
figure;
histogram(data.surviving);
xlabel("Number of Agents Survived");
ylabel("Number of Simulations");

%% Percent of Surviving Agents
figure;
survivingPercent = data.surviving ./ data.numAgents * 100;
histogram(survivingPercent,'BinEdges',(0:100/20:100));
xlabel("Percent of Surviving Agents");
ylabel("Number of Simulations")
set(gca,'fontsize',20);

%% Print Percents Surviving
thresholds = [100,75,50,10,1];
for i=1:length(thresholds)
    threshold = thresholds(i);
    num = sum(survivingPercent>=threshold);
    percent = num/length(survivingPercent)*100;
    fprintf("%g simulations (%.2f%%) had at least %g%% of agents survive.\n",num,percent,threshold);
end
    fprintf("%g simulations (%.2f%%) had no agents survive.\n",sum(survivingPercent == 0),sum(survivingPercent == 0)/length(survivingPercent)*100);