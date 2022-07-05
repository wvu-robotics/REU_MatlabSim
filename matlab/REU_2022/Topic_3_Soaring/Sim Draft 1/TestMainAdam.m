%% Prepare Workspace
close all;
clear;
clc;

%% Read in SimLaw from Excel Sheet
addpath("Code of Laws");
simLawExcel = "AdamsLaw.xlsx";
[~,~,RAW] = xlsread(simLawExcel);
varLabelColumn = 2;
startingColumn = 3;
startingRow = 1;
numSims = size(RAW,2) - startingColumn + 1;

% Parse data
for sim = 1:numSims
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
        BigSL(sim).(varLabel) = varValue;
    end
end

SL = BigSL(1);
