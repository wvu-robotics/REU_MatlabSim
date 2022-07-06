%% Prepare Workspace
close all;
clear;
clc;

%% Generate unique code for simulation batch, from date
simBatchCode = datestr(now,"yyyymmddHHMMSS");

%% Add search paths for sim laws and agent functions
addpath("Code of Laws");
addpath("Agent Control Functions");
addpath("Find Neighborhood Functions");

%% setup output folders
%Output Media folder
rootFolder = "Output Media";
if(~exist(rootFolder,'dir'))
    mkdir(rootFolder);
end

%Date folder
date = datestr(now,"mm-dd-yy");
dateFolder = sprintf('%s/%s',rootFolder,date);
if(~exist(dateFolder,'dir'))
    mkdir(dateFolder);
end

%Sim Batch folder
simBatchFolder = sprintf('%s/SimBatch %s',dateFolder,simBatchCode);
if(~exist(simBatchFolder,'dir'))
    mkdir(simBatchFolder);
end

%% Set up input Excel sheet
simLawExcel = "AdamsLaw.xlsx";
sheetNum = 1;
[~,~,RAW] = xlsread(simLawExcel,sheetNum);
varLabelColumn = 2;
startingColumn = 3;
startingRow = 1;
numSims = size(RAW,2) - startingColumn + 1;

%% Set up output Excel sheet
outputExcelName = sprintf('%s/SimData %s.xlsx',simBatchFolder,simBatchCode);
% Copy original excel sheet
xlswrite(outputExcelName,RAW,sheetNum,'A1');
outputTitleRow = size(RAW,1) + 5;
outputTitlePos = sprintf('A%d',outputTitleRow);
% Write label 'OUTPUT'
xlswrite(outputExcelName,{'OUTPUT'},sheetNum,outputTitlePos);
outputRow = outputTitleRow + 1;
outputVariables = ["timeStart";"timeEnd";"surviving";"flightTime";"finalHeightMax";"finalHeightMin";"finalHeightAvg"];
outputLabelPos = sprintf('A%d',outputRow);
% Write labels of output variables
xlswrite(outputExcelName,outputVariables,sheetNum,outputLabelPos);

%{
fprintf("Done!\n");
pause(5);
pause(1000);
%}

%% Iterate for each simulation excel column
for sim = 1:numSims
    % Parse current SimLaw
    simColumn = startingColumn+sim-1;
    for varRow = startingRow:size(RAW,1)
        varLabel = RAW{varRow,varLabelColumn};
        if(isnan(varLabel))
            continue;
        end
        varValue = RAW{varRow,simColumn};
        if(class(varValue) == "char")
            %Try evaluating string
            try
                varValue = eval(varValue);
            catch
                % Do nothing, keep as string
            end
        end
        SL.(varLabel) = varValue;
    end
    
    %% Run Simulation
    simNumber = SL.simIDNum;
    render = true;
    videoName = sprintf('%s/%d SimRender.avi',simBatchFolder,simNumber);
    outputData = MainScriptFunction(SL, simNumber, videoName, render);
    
    %% Store sim output data
    fprintf("Storing results of sim %d.\n",simNumber);
    for varIndex = 1:length(outputVariables)
        varValue = outputData.(outputVariables(varIndex));
        
        varColumn = char(simColumn + 64);
        varRow = outputRow + varIndex - 1;
        varPos = sprintf("%s%d",varColumn,varRow);
        
        xlswrite(outputExcelName,{varValue},sheetNum,varPos);
    end
    fprintf("Finished storing results of sim %d.\n",simNumber);
end





