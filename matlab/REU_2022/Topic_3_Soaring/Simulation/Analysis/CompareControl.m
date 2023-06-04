%% Prepare workspace
close all
clear
clc

%% Load data
fileName = "..\Megaruns\Megarun_7\7-25-22\CombinedData_7_25_22.mat";
varNames = ["cohesion","separation","alignment","cohPower","migration","numThermals","numAgents","rngSeed","funcName_agentControl"];
data = load(fileName);

%% Process data
%{

migration = 1e-21
numThermals > 3
numAgents = 40

%}


indBaseline = (data.funcName_agentControl == "agentControl_Baseline");
indUnified = (data.funcName_agentControl == "agentControl_Unified");

indMig = (data.migration == 1e-21);
indTherm = (data.numThermals > 3);
indAgents = (data.numAgents == 40);

filter = indMig & indTherm & indAgents;

%% Render histogram(s)
renderHist(data,indBaseline,indUnified,"Unfiltered",18000);
renderHist(data,filter & indBaseline,filter & indUnified,"Filtered18000",18000);
renderHist(data,filter & indBaseline,filter & indUnified,"Filtered2500",2500);
renderHist(data,filter & indBaseline,filter & indUnified,"Filtered500",500);


function renderHist(data,indLeft,indRight,figName,maxY)
    binEdges = 0:41;
    xlims = [0,40];
    fontSizeTitle = 40;
    fontSizeAxes = 30;
    xLabelString = "Number of Survivors";
    yLabelString = "Number of Sims";
    
    figure('NumberTitle','off','Name',figName);
    tiledlayout(1,2);

    nexttile;
    histogram(data.surviving(indLeft),'BinEdges',binEdges);
    xlim(xlims);
    if(~isnan(maxY))
        ylim([0,maxY]);
    end
    title("Baseline Control",'FontSize',fontSizeTitle);
    xlabel(xLabelString,'FontSize',fontSizeAxes);
    ylabel(yLabelString,'FontSize',fontSizeAxes);
    
    nexttile;
    histogram(data.surviving(indRight),'BinEdges',binEdges);
    xlim(xlims);
    if(~isnan(maxY))
        ylim([0,maxY]);
    end
    title("Time-Varying Control",'FontSize',fontSizeTitle);
    xlabel(xLabelString,'FontSize',fontSizeAxes);
    ylabel(yLabelString,'FontSize',fontSizeAxes);
end