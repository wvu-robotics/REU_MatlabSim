%% Prepare workspace
%close all
clear
clc

%% Load data
run = 4;
if(run == 23)
    fileName = "..\Megaruns\Megarun_2-3\7-1920-22\CombinedData_7_1920_22.mat";
    varNames = ["rngSeed","cohesion","heightFactorPower","cohesionAscensionIgnore","cohesionAscensionMax","ascensionFactorPower","separation","alignment"];
elseif(run == 4)
    fileName = "..\Megaruns\Megarun_4\7-21-22\CombinedData_7_21_22.mat";
    varNames = ["rngSeed","cohesion","cohesionAscensionIgnore","cohPower","separation","alignment","k"];
elseif(run == 5)
    fileName = "..\Megaruns\Megarun_5\7-22-22\CombinedData_7_22_22.mat";
    varNames = ["rngSeed","cohesion","cohPower","separation","alignment","separationHeightWidth","alignmentHeightWidth"];
end
data = load(fileName);

%% Display data

allNames = ["rngSeed","cohesion","heightFactorPower","cohesionAscensionIgnore","cohesionAscensionMax","cohPower","separation","alignment","separationHeightWidth","alignmentHeightWidth","k"];
allScales = ["linear","log","linear","linear","linear","linear","log","log","linear","linear","linear"];
allNicknames = ["rng","coh","hgtPow","RAIgn","RAMax","cohPow","sep","align","sepGap","alignGap","k"];

scaleMap = containers.Map(allNames,allScales);
nameMap = containers.Map(allNames,allNicknames);

for i=1:(length(varNames)-1)
    for j=i+1:length(varNames)
        %Run comparison of vars i vs j
        plotComparison(data,varNames(i),scaleMap(varNames(i)),varNames(j),scaleMap(varNames(j)),"surviving","heightScore","explorationPercent","thermalUseScore",nameMap);
    end
end


%% Good func
function plotComparison(data,indep1,indep1Scale,indep2,indep2Scale,dep1,dep2,dep3,dep4,nameMap)
    figure('NumberTitle','off','Name',sprintf("%s VS %s",nameMap(indep1),nameMap(indep2)));
    fprintf("Plotted %s vs %s.\n",indep1,indep2);
    tiledlayout(2,2);
    alpha = 1;
    
    nextplot(data,indep1,indep1Scale,indep2,indep2Scale,dep1,alpha);
    nextplot(data,indep1,indep1Scale,indep2,indep2Scale,dep2,alpha);
    nextplot(data,indep1,indep1Scale,indep2,indep2Scale,dep3,alpha);
    nextplot(data,indep1,indep1Scale,indep2,indep2Scale,dep4,alpha);
end

function nextplot(data,indep1Name,indep1Scale,indep2Name,indep2Scale,depName,alpha)
    [UI1,UI2,avgDep] = avgData(data,indep1Name,indep2Name,depName);

    nexttile;
    %scatter3(UI1,UI2,avgDep,'Marker','o','MarkerEdgeAlpha',alpha,'MarkerFaceAlpha',alpha);
    surf(UI1,UI2,avgDep,'FaceAlpha',alpha);
    set(gca,'XScale',indep1Scale);
    set(gca,'YScale',indep2Scale);
    xlabel(indep1Name);
    ylabel(indep2Name);
    zlabel(depName);
end

function [UI1,UI2,avgDep] = avgData(data,indep1Name,indep2Name,depName)
    indep1 = data.(indep1Name);
    indep2 = data.(indep2Name);
    dep = data.(depName);
    unique_indep1 = unique(indep1);
    unique_indep2 = unique(indep2);
    numUI1 = length(unique_indep1);
    numUI2 = length(unique_indep2);
    
    sums = cell(numUI1,numUI2);
    valueMapUI1 = containers.Map(unique_indep1,1:numUI1);
    valueMapUI2 = containers.Map(unique_indep2,1:numUI2);
    
    for i=1:length(dep)
        indexUI1 = valueMapUI1(indep1(i));
        indexUI2 = valueMapUI2(indep2(i));
        sums{indexUI1,indexUI2} = [sums{indexUI1,indexUI2},dep(i)];
    end
    
    averages = cell(numUI1,numUI2);
    for i=1:numUI1
        for j=1:numUI2
            averages{i,j} = sum(sums{i,j})/length(sums{i,j});
        end
    end
    
    avgMat = cell2mat(averages);
    [UI2Mat,UI1Mat] = meshgrid(unique_indep2,unique_indep1);
    
    UI1 = UI1Mat;
    UI2 = UI2Mat;
    avgDep = avgMat;
    
    %UI1 = reshape(UI1Mat,[1,numUI1*numUI2]);
    %UI2 = reshape(UI2Mat,[1,numUI1*numUI2]);
    %avgDep = reshape(avgMat,[1,numUI1*numUI2]);
end