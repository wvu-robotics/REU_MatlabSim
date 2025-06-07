%% Prepare workspace
close all
clear
clc

%% Load data
run = 7.1;
if(run == 23)
    fileName = "..\Megaruns\Megarun_2-3\7-1920-22\CombinedData_7_1920_22.mat";
    varNames = ["rngSeed","cohesion","heightFactorPower","cohesionAscensionIgnore","cohesionAscensionMax","ascensionFactorPower","separation","alignment"];
elseif(run == 4)
    fileName = "..\Megaruns\Megarun_4\7-21-22\CombinedData_7_21_22.mat";
    varNames = ["rngSeed","cohesion","cohesionAscensionIgnore","cohPower","separation","alignment","k"];
elseif(run == 5)
    fileName = "..\Megaruns\Megarun_5\7-22-22\CombinedData_7_22_22.mat";
    varNames = ["rngSeed","cohesion","cohPower","separation","alignment","separationHeightWidth","alignmentHeightWidth"];
elseif(run == 7)
    fileName = "..\Megaruns\Megarun_7\7-25-22\CombinedData_7_25_22.mat";
    varNames = ["cohesion","separation","alignment","cohPower","migration","numThermals","numAgents","rngSeed","funcName_agentControl"];
elseif(run == 7.1)
    fileName = "Megarun Data\Megarun7Unified.mat";
    varNames = ["cohesion","separation","alignment","cohPower","migration","numThermals","numAgents","rngSeed"];
end
data = load(fileName);

%% Display data

allNames = ["rngSeed","cohesion","heightFactorPower","cohesionAscensionIgnore","cohesionAscensionMax","ascensionFactorPower","cohPower","separation","alignment","separationHeightWidth","alignmentHeightWidth","k","migration","numThermals","numAgents","funcName_agentControl"];
allScales = ["linear","log","linear","linear","linear","linear","linear","log","linear","linear","linear","linear","log","linear","linear","linear"];
allNicknames = ["rng","coh","hgtPow","RAIgn","RAMax","RAPow","cohPow","sep","align","sepGap","alignGap","k","mig","#Thrms","#Agts","Ctrl"];

scaleMap = containers.Map(allNames,allScales);
nameMap = containers.Map(allNames,allNicknames);

for i=1:length(varNames)
    figure;
    title(varNames(i));
    tiledlayout(2,2);

    nexttile;
    scatter(data.(varNames(i)),data.surviving);
    xlabel(varNames(i));
    set(gca,'XScale',scaleMap(varNames(i)));
    ylabel("Surviving");
    
    nexttile;
    scatter(data.(varNames(i)),data.heightScore);
    xlabel(varNames(i));
    set(gca,'XScale',scaleMap(varNames(i)));
    ylabel("HeightScore");
    
    nexttile;
    scatter(data.(varNames(i)),data.explorationPercent);
    xlabel(varNames(i));
    set(gca,'XScale',scaleMap(varNames(i)));
    ylabel("ExplorationPercent");
    
    nexttile;
    scatter(data.(varNames(i)),data.thermalUseScore);
    xlabel(varNames(i));
    set(gca,'XScale',scaleMap(varNames(i)));
    ylabel("ThermalUseScore");
    
    fprintf("Finished plotting %s.\n",varNames(i));
end
return

for i=1:length(varNames)
    plotComparison(data,varNames(i),scaleMap(varNames(i)),"surviving","heightScore","explorationPercent","thermalUseScore",nameMap);
end


%% Good func
function plotComparison(data,indep1,indep1Scale,dep1,dep2,dep3,dep4,nameMap)
    figure('NumberTitle','off','Name',nameMap(indep1));
    fprintf("Plotted %s\n",indep1);
    tiledlayout(2,2);
    
    nextplot(data,indep1,indep1Scale,dep1);
    nextplot(data,indep1,indep1Scale,dep2);
    nextplot(data,indep1,indep1Scale,dep3);
    nextplot(data,indep1,indep1Scale,dep4);
end

function nextplot(data,indep1Name,indep1Scale,depName)
    [UI1,avgDep] = avgData(data,indep1Name,depName);

    nexttile;
    %scatter3(UI1,UI2,avgDep,'Marker','o','MarkerEdgeAlpha',alpha,'MarkerFaceAlpha',alpha);
    xTick = NaN;
    yTick = NaN;
    if(class(UI1) == "string")
        valueMap = containers.Map(unique(UI1),1:length(unique(UI1)));
        xTick = 1:size(UI1,1);
        xTickLabel = strings(1,length(xTick));
        xTickLabel(xTick) = UI1(xTick,1);
        newUI1 = zeros(size(UI1));
        for i=1:numel(UI1)
            newUI1(i) = valueMap(UI1(i));
        end
        UI1 = newUI1;
        set(gca,'XTick',xTick,'XTickLabel',xTickLabel);
    end
    plot(UI1,avgDep)
    if(~isnan(xTick))
        set(gca,'XTick',xTick,'XTickLabel',xTickLabel);
    end
    if(~isnan(yTick))
        set(gca,'YTick',yTick,'YTickLabel',yTickLabel);
    end
    set(gca,'XScale',indep1Scale);
    xlabel(indep1Name);
    ylabel(depName);
end

function [UI1,avgDep] = avgData(data,indep1Name,depName)
    indep1 = data.(indep1Name);
    dep = data.(depName);
    unique_indep1 = unique(indep1);
    numUI1 = length(unique_indep1);
    
    sums = cell(numUI1,1);
    valueMapUI1 = containers.Map(unique_indep1,1:numUI1);
    
    for i=1:length(dep)
        indexUI1 = valueMapUI1(indep1(i));
        sums{indexUI1,1} = [sums{indexUI1,1},dep(i)];
    end
    
    averages = cell(numUI1,1);
    for i=1:numUI1
        averages{i,1} = sum(sums{i,1})/length(sums{i,1});
    end
    
    avgMat = cell2mat(averages);
    
    UI1 = unique_indep1;
    avgDep = avgMat;
end