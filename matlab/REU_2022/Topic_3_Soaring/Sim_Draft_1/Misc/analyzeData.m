%% Prepare workspace
%close all
clear
clc

%% Load data
fileName = "E:\WVU_REU\7-20-22\CombinedData_7_1920_22.mat";
data = load(fileName);

%% Display data
%labels = {'rngSeed','cohesion','heightFactorPower','cohesionAscensionIgnore','cohesionAscensionMax','ascensionFactorPower','Separation','Alignment'};
varName = "ascensionFactorPower";
varScale = "linear";
plotComparison(data,"rngSeed","linear",varName,varScale,"surviving","heightScore","explorationPercent","thermalUseScore");
plotComparison(data,"cohesion","log",varName,varScale,"surviving","heightScore","explorationPercent","thermalUseScore");
plotComparison(data,"heightFactorPower","linear",varName,varScale,"surviving","heightScore","explorationPercent","thermalUseScore");
plotComparison(data,"cohesionAscensionIgnore","linear",varName,varScale,"surviving","heightScore","explorationPercent","thermalUseScore");
plotComparison(data,"cohesionAscensionMax","linear",varName,varScale,"surviving","heightScore","explorationPercent","thermalUseScore");
plotComparison(data,"ascensionFactorPower","linear",varName,varScale,"surviving","heightScore","explorationPercent","thermalUseScore");
plotComparison(data,"separation","log",varName,varScale,"surviving","heightScore","explorationPercent","thermalUseScore");
plotComparison(data,"alignment","log",varName,varScale,"surviving","heightScore","explorationPercent","thermalUseScore");

%% Good func
function plotComparison(data,indep1,indep1Scale,indep2,indep2Scale,dep1,dep2,dep3,dep4)
    figure('NumberTitle','off','Name',sprintf("%s VS %s",indep1,indep2));
    tiledlayout(2,2);
    alpha = 1;
    
    nextplot(data,indep1,indep1Scale,indep2,indep2Scale,dep1,alpha);
    nextplot(data,indep1,indep1Scale,indep2,indep2Scale,dep2,alpha);
    nextplot(data,indep1,indep1Scale,indep2,indep2Scale,dep3,alpha);
    nextplot(data,indep1,indep1Scale,indep2,indep2Scale,dep4,alpha);
    
    %{
    nexttile;
    scatter3(data.(indep1),data.(indep2),data.(dep1),'Marker','.','MarkerEdgeAlpha',alpha,'MarkerFaceAlpha',alpha);
    set(gca,'XScale',indep1Scale);
    set(gca,'YScale',indep2Scale);
    xlabel(indep1);
    ylabel(indep2);
    zlabel(dep1);
    
    nexttile;
    scatter3(data.(indep1),data.(indep2),data.(dep2),'Marker','.','MarkerEdgeAlpha',alpha,'MarkerFaceAlpha',alpha);
    set(gca,'XScale',indep1Scale);
    set(gca,'YScale',indep2Scale);
    xlabel(indep1);
    ylabel(indep2);
    zlabel(dep2);
    
    nexttile;
    scatter3(data.(indep1),data.(indep2),data.(dep3),'Marker','.','MarkerEdgeAlpha',alpha,'MarkerFaceAlpha',alpha);
    set(gca,'XScale',indep1Scale);
    set(gca,'YScale',indep2Scale);
    xlabel(indep1);
    ylabel(indep2);
    zlabel(dep3);
    
    nexttile;
    scatter3(data.(indep1),data.(indep2),data.(dep4),'Marker','.','MarkerEdgeAlpha',alpha,'MarkerFaceAlpha',alpha);
    set(gca,'XScale',indep1Scale);
    set(gca,'YScale',indep2Scale);
    xlabel(indep1);
    ylabel(indep2);
    zlabel(dep4);
    %}
end

function nextplot(data,indep1Name,indep1Scale,indep2Name,indep2Scale,depName,alpha)
    [UI1,UI2,avgDep] = avgData(data,indep1Name,indep2Name,depName);

    nexttile;
    %scatter3(UI1,UI2,avgDep,'Marker','o','MarkerEdgeAlpha',alpha,'MarkerFaceAlpha',alpha);
    surf(UI1,UI2,avgDep);
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

%% Comments
%{
Small:

scatter3(idata.cohesion,idata.separation,odata.surviving);
set(gca,'XScale','log');
set(gca,'YScale','log');
xlabel('Cohesion');
ylabel('Separation');
zlabel('Surviving');


Big:
clf;
tiledlayout(2,2);
nexttile;
scatter3(idata.cohesion,idata.separation,odata.surviving);
set(gca,'XScale','log');
set(gca,'YScale','log');
xlabel('Cohesion');
ylabel('Separation');
zlabel('Surviving');
nexttile;
scatter3(idata.cohesion,idata.separation,odata.heightScore);
set(gca,'XScale','log');
set(gca,'YScale','log');
xlabel('Cohesion');
ylabel('Separation');
zlabel('Height Score');
nexttile;
scatter3(idata.cohesion,idata.separation,odata.explorationPercent);
set(gca,'XScale','log');
set(gca,'YScale','log');
xlabel('Cohesion');
ylabel('Separation');
zlabel('Exploration Percent');
nexttile;
scatter3(idata.cohesion,idata.separation,odata.thermalUseScore);
set(gca,'XScale','log');
set(gca,'YScale','log');
xlabel('Cohesion');
ylabel('Separation');
zlabel('Thermal Use Score');
%}
