clc; clear; close all
%addpath("Megarun Data");
%mr7 = load("Megarun7.mat");

%% Load data
fileName = "..\Megaruns\Megarun_7\7-25-22\CombinedData_7_25_22.mat";
varNames = ["cohesion","separation","alignment","cohPower","migration","numThermals","numAgents","rngSeed","funcName_agentControl"];
data = load(fileName);
%data = mr7;

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
renderHist(data,indBaseline,indUnified,"Unfiltered500",500);
renderHistTogether(data,indBaseline,indUnified,"Unfiltered",18000);
renderHistTogether(data,indBaseline,indUnified,"Unfiltered500",500);
%renderHist(data,filter & indBaseline,filter & indUnified,"Filtered18000",18000);
%renderHist(data,filter & indBaseline,filter & indUnified,"Filtered2500",2500);
%renderHist(data,filter & indBaseline,filter & indUnified,"Filtered500",500);


function renderHist(data,indLeft,indRight,figName,maxY)
    binEdges = 0:41;
    xlims = [0,40];
    fontSizeTitle = 30;
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
    title("Unmodified Boids Controller",'FontSize',fontSizeTitle);
    xlabel(xLabelString,'FontSize',fontSizeAxes);
    ylabel(yLabelString,'FontSize',fontSizeAxes);
    
    nexttile;
    histogram(data.surviving(indRight),'BinEdges',binEdges);
    xlim(xlims);
    if(~isnan(maxY))
        ylim([0,maxY]);
    end
    title("Modified Boids Controller",'FontSize',fontSizeTitle);
    xlabel(xLabelString,'FontSize',fontSizeAxes);
    ylabel(yLabelString,'FontSize',fontSizeAxes);
end

function renderHistTogether(data,indLeft,indRight,figName,maxY)
    binEdges = 0:41;
    xlims = [0,40];
    fontSizeTitle = 30;
    fontSizeAxes = 30;
    xLabelString = "Number of Survivors";
    yLabelString = "Number of Sims";
    faceAlpha = 0.5;
    
    figure('NumberTitle','off','Name',figName);
    
    hold on
    histogram(data.surviving(indRight),'BinEdges',binEdges,'FaceColor',[1,0,0],'FaceAlpha',faceAlpha);
    histogram(data.surviving(indLeft),'BinEdges',binEdges,'FaceColor',[0,0,1],'FaceAlpha',faceAlpha);
    xlim(xlims);
    if(~isnan(maxY))
        ylim([0,maxY]);
    end
    xlabel(xLabelString,'FontSize',fontSizeAxes);
    ylabel(yLabelString,'FontSize',fontSizeAxes);
    legend('Modified Boids Controller','Unmodified Boids Controller')
end