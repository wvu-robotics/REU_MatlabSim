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
RAW = readcell(simLawExcel,'Sheet',sheetNum);
for i=1:size(RAW,1)
    for j=1:size(RAW,2)
        if(ismissing(RAW{i,j}))
            RAW{i,j} = NaN;
        end
    end
end
varLabelColumn = 2;
startingColumn = 3;
startingRow = 1;
numColumns = size(RAW,2) - startingColumn + 1;
% Find valid sim columns
numSims = 0;
simColumns = NaN(1,numColumns); % Preallocate max number of valid columns
for col = 1:numColumns
    %try
        if (~isnan(RAW{startingRow,col+startingColumn-1}))
            numSims = numSims + 1;
            simColumns(numSims) = col+startingColumn-1;
        end
    %catch
    %    fprintf("adw\n");
    %end
end
simColumns = simColumns(1:numSims); % Trim excess NaNs

%% Set up output Excel sheet
outputExcelName = sprintf('%s/SimData_%s.xlsx',simBatchFolder,simBatchCode);
% Copy original excel sheet
writecell(RAW,outputExcelName,'Sheet',sheetNum,'Range','A1','AutoFitWidth',0);
outputTitleRow = size(RAW,1) + 5;
% Write label 'OUTPUT'
writecell({'OUTPUT'},outputExcelName,'Sheet',sheetNum,'Range',Utility.findSSPos(outputTitleRow,1),'AutoFitWidth',0);
% Write labels of output variables
outputVariables = ["simNumber";"timeStart";"timeEnd";"surviving";"flightTime";"finalHeightMax";"finalHeightMin";"finalHeightAvg"];
outputRow = outputTitleRow + 1;
writematrix(outputVariables,outputExcelName,'Sheet',sheetNum,'Range',Utility.findSSPos(outputRow,varLabelColumn));

%% Iterate for each simulation excel column
fprintf("Starting %g sim(s).\n",numSims);
parfor sim = 1:numSims
    % Parse current SimLaw
    simColumn = simColumns(sim);
    SL = struct();
    for varRow = 1:size(RAW,1)
        varLabel = RAW{varRow,varLabelColumn};
        if(isnan(varLabel))
            continue;
        end
        varValue = RAW{varRow,simColumn};
        if(class(varValue) == "char")
            %Try evaluating string
            varValue = Utility.parTryEval(varValue);
        end
        if(varLabel == "Show")
            fprintf("%s\n",varValue);
            varValue
        end
        SL.(varLabel) = varValue;
    end
    
    %% Run Simulation
    simNumber = SL.simIDNum;
    videoName = sprintf('%s/%d_SimRender.avi',simBatchFolder,simNumber);
    outputData = MainScriptFunction(SL, simNumber, videoName);
    bigOutputData(sim) = outputData;
    fprintf("Finished sim %d.\n",simNumber);
    
    %% Save sim outputData to .mat file
    outputDataName = sprintf("%s/%d_OutputData.mat",simBatchFolder,simNumber);
    Utility.parSave(outputDataName,outputData);
end

%% Save data
storeData = cell(length(outputVariables),numColumns);
for sim = 1:numSims
    %% Parse bigOutputData into cell array
    outputData = bigOutputData(sim);
    simNumber = outputData.simNumber;
    simColumn = simColumns(sim);
    for varIndex = 1:length(outputVariables)
        varValue = outputData.(outputVariables(varIndex));
        storeData(varIndex,simColumn-startingColumn+1) = {varValue};
    end
end

%% Save cell array to excel sheet
writecell(storeData,outputExcelName,'Sheet',sheetNum,'Range',Utility.findSSPos(outputRow,startingColumn),'AutoFitWidth',0);
fprintf("Finished storing results.\n");