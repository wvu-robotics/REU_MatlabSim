%% Prepare Workspace
close all;
clear;
clc;

%% Choose sim law
simLawExcel = "Law_40SurvivorsExample.xlsx";
sheetNum = 1;

%% Generate unique code for simulation batch, from date
simBatchCode = datestr(now,"yyyymmddHHMMSS");

%% Add search paths for sim laws and agent functions
addpath("Code_of_Laws");
addpath("Agent_Control_Functions");
addpath("Find_Neighborhood_Functions");

%% Setup output folders
% Output Media folder
outputFolderRoot = "../Output_Media";
if(~exist(outputFolderRoot,'dir'))
    mkdir(outputFolderRoot);
end

% Date folder
date = datestr(now,"mm-dd-yy");
outputFolderDate = sprintf('%s/%s',outputFolderRoot,date);
if(~exist(outputFolderDate,'dir'))
    mkdir(outputFolderDate);
end

% Sim Batch folder
simBatchFolder = sprintf('%s/SimBatch_%s',outputFolderDate,simBatchCode);
if(~exist(simBatchFolder,'dir'))
    mkdir(simBatchFolder);
end

% Mat files folder
matFilesFolder = sprintf('%s/MatFiles',simBatchFolder);
if(~exist(matFilesFolder,'dir'))
    mkdir(matFilesFolder);
end

%% Parse sim law Excel sheet
fprintf("Parsing Excel sheet... ");
RAW = readcell(simLawExcel,'Sheet',sheetNum);
% Convert missing to NaN
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
fprintf("Done!\n");

%% Prepare large SL with all iterations
fprintf("Preparing SL for %d combinations... ",combinations);
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
fprintf("Done!\n");

% Print variables to iterate
if(~isempty(changedVariables))
    fprintf("Ready to iterate over variables:\n");
    for i=1:length(changedVariables)
        fprintf(" - %s\n",changedVariables(i));
    end
else
    fprintf("No variables to iterate over.\n");
end


%% Iterate through simulations
fprintf("Starting simulations for %g combination(s).\n",combinations);
numSims = combinations;
for sim = 1:numSims
    %% Run Simulation
    SL = SLIterations(sim);
    simNumber = sim;
    videoName = sprintf('%s/%d_SimRender.avi',simBatchFolder,simNumber);
    outputData = MainScriptFunction(SL, simNumber, videoName);
    % Store simulation output in big struct array
    % Not preallocated because MainScriptFunction determines struct headers
    bigOutputData(sim) = outputData; %#ok<SAGROW> 
    fprintf("Finished sim %d.\n",simNumber);
    
    %% Save sim outputData to .mat file
    outputDataName = sprintf("%s/%d_OutputData.mat",matFilesFolder,simNumber);
    Utility.parSave(outputDataName,outputData);
end

fprintf("Finished simulations.\n");

%% Combine all output data into single file
fprintf("Combining files... ");
Utility.combineMatFiles(matFilesFolder, changedVariables, simBatchCode);
fprintf("Done!\n");

%% Finished simulation
fprintf("Done!\n");