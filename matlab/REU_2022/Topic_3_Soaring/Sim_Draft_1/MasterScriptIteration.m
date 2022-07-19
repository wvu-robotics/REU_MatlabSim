%% Prepare Workspace
close all;
clear;
clc;

%% Generate unique code for simulation batch, from date
simBatchCode = datestr(now,"yyyymmddHHMMSS");

%% Add search paths for sim laws and agent functions
addpath("Code_of_Laws");
addpath("Agent_Control_Functions");
addpath("Find_Neighborhood_Functions");
addpath("Misc");

%% setup output folders
%Output Media folder
rootFolder = "Output_Media";
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
simBatchFolder = sprintf('%s/SimBatch_%s',dateFolder,simBatchCode);
if(~exist(simBatchFolder,'dir'))
    mkdir(simBatchFolder);
end

%Mat files folder
matFilesFolder = sprintf('%s/MatFiles',simBatchFolder);
if(~exist(matFilesFolder,'dir'))
    mkdir(matFilesFolder);
end

%% Set up input Excel sheet
simLawExcel = "AdamsLawIteration.xlsx";
sheetNum = 1;
RAW = readcell(simLawExcel,'Sheet',sheetNum);
% Convert missing to NaN
for i=1:size(RAW,1)
    for j=1:size(RAW,2)
        if(ismissing(RAW{i,j}))
            RAW{i,j} = NaN;
        end
    end
end

%% Parse input Excel sheet
fprintf("Reading Excel sheet.\n");
varLabelColumn = 2;
startingColumn = 3;
startingRow = 1;
numColumns = size(RAW,2) - startingColumn + 1;

BigSL = struct();
combinations = 1;
changedVariables = strings(0);
for row = startingRow:size(RAW,1)
    % For each variable
    varLabel = RAW{row,varLabelColumn};
    if(isnan(varLabel))
        continue;
    end
    numValues = 0;
    for column = startingColumn:size(RAW,2)
        % For each variable value
        varValue = RAW{row,column};
        if(isnan(varValue))
            continue;
        end
        if(class(varValue) == "char")
            %Try evaluating string
            varValue = Utility.parTryEval(varValue);
        end
        
        childVarLabel = sprintf("%s%d",varLabel,column);
        BigSL.(varLabel).(childVarLabel) = varValue;
        numValues = numValues + 1;
    end
    if(numValues > 1)
        combinations = combinations * numValues;
        changedVariables(length(changedVariables)+1) = varLabel;
    end
end

%% Prepare large SL with all iterations
fprintf("Preparing SL for %d combinations.\n",combinations);
SLIterations(1:combinations) = struct();
repeat = 1;

varLabels = fieldnames(BigSL);
for i=1:size(varLabels,1)
    varLabel = varLabels{i};
    valueLabels = fieldnames(BigSL.(varLabel));
    numValues = size(valueLabels,1);
    numCycles = combinations / (repeat * numValues);
    storeIndex = 1;
    for j=1:numCycles
        for k=1:numValues
            valueLabel = valueLabels{k};
            for l=1:repeat
                SLIterations(storeIndex).(varLabel) = BigSL.(varLabel).(valueLabel);
                storeIndex = storeIndex + 1;
            end
        end
    end
    repeat = repeat * numValues;
end

fprintf("Done. Ready to iterate over variables:\n");
for i=1:length(changedVariables)
    fprintf(" - %s\n",changedVariables(i));
end

%% Iterate through simulations
fprintf("Starting simulations for %g combination(s).\n",combinations);
numSims = combinations;
parfor sim = 1:numSims
    %% Run Simulation
    SL = SLIterations(sim);
    simNumber = sim;
    videoName = sprintf('%s/%d_SimRender.avi',simBatchFolder,simNumber);
    outputData = MainScriptFunction(SL, simNumber, videoName);
    bigOutputData(sim) = outputData;
    fprintf("Finished sim %d.\n",simNumber);
    
    %% Save sim outputData to .mat file
    outputDataName = sprintf("%s/%d_OutputData.mat",matFilesFolder,simNumber);
    Utility.parSave(outputDataName,outputData);
end

fprintf("Finished simulations.\n");

%% Save data to Excel sheet
outputExcelName = sprintf('%s/SimData_%s.xlsx',simBatchFolder,simBatchCode);
outputVariables = ["simNumber";"rngSeed";"timeStart";"timeEnd";"surviving";"collisionDeaths";"groundDeaths";"flightTime";"heightScore";"explorationPercent";"thermalUseScore";"finalHeightMax";"finalHeightMin";"finalHeightAvg"];
Utility.generateOutputExcelSheet(outputExcelName,matFilesFolder,RAW,changedVariables,outputVariables);
fprintf("Done!\n");

%% microwave
microwave = false;
if(microwave)
    [y,Fs] = audioread("microwave.mp3");
    sound(y,Fs)
end