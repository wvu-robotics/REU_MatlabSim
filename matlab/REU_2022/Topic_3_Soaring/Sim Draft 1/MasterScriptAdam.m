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
nanCount = 0;
for col = 1:numSims
    if isnan(RAW{1,col+startingColumn-1})
        nanCount = nanCount + 1;
    end
end
numSims = numSims - nanCount;

%% Set up output Excel sheet
outputExcelName = sprintf('%s/SimData %s.xlsx',simBatchFolder,simBatchCode);
% Copy original excel sheet
xlswrite(outputExcelName,RAW,sheetNum,'A1');
outputTitleRow = size(RAW,1) + 5;
outputTitlePos = sprintf('A%d',outputTitleRow);
% Write label 'OUTPUT'
xlswrite(outputExcelName,{'OUTPUT'},sheetNum,outputTitlePos);
outputRow = outputTitleRow + 1;
outputVariables = ["simNumber";"timeStart";"timeEnd";"surviving";"flightTime";"finalHeightMax";"finalHeightMin";"finalHeightAvg"];
outputLabelPos = sprintf('A%d',outputRow);
% Write labels of output variables
xlswrite(outputExcelName,outputVariables,sheetNum,outputLabelPos);

%% Iterate for each simulation excel column
%bigOutputData(1:numSims) = struct();
parfor sim = 1:numSims
    % Parse current SimLaw
    simColumn = startingColumn+sim-1;
    SL = struct();
    for varRow = 1:size(RAW,1)
        varLabel = RAW{varRow,varLabelColumn};
        if(isnan(varLabel))
            continue;
        end
        varValue = RAW{varRow,simColumn};
        fprintf("Val: %s=%g\n",varValue,varValue);
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
    render = SL.render;
    videoName = sprintf('%s/%d SimRender.avi',simBatchFolder,simNumber);
    outputData = MainScriptFunction(SL, simNumber, videoName, render);
    bigOutputData(sim) = outputData;
    fprintf("Finished sim %d.\n",simNumber);
    
    fprintf("IsProp: %g\n",isprop(SL,"rngSeed"));
    if(isprop(SL,"rngSeed"))
        fprintf("Prop: %g\n",SL.rngSeed);
        fprintf("IsNan: %g\n",isnan(SL.rngSeed));
    end
end

%% Save data
storeData = cell(length(outputVariables),numSims);
for sim = 1:numSims
    %% Parse bigOutputData into cell array
    outputData = bigOutputData(sim);
    simNumber = outputData.simNumber;
    for varIndex = 1:length(outputVariables)
        varValue = outputData.(outputVariables(varIndex));
        storeData(varIndex,sim) = {varValue};
    end
    
    %% Save sim outputData to .mat file
    outputDataName = sprintf("%s/%d OutputData.mat",simBatchFolder,simNumber);
    save(outputDataName,'-struct','outputData');
end

%% Save cell array to excel sheet
storeColumn = char(startingColumn + 64);
storeRow = outputRow;
storePos = sprintf("%s%d",storeColumn,storeRow);
xlswrite(outputExcelName,storeData,sheetNum,storePos);    
fprintf("Finished storing results.\n");