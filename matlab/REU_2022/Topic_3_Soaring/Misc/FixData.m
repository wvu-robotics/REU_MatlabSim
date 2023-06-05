%% Prepare Workspace
clc;
clear;
close all;

%% Define path to target .mat file
megarun = 3;

filePaths = ["";
             "../Megaruns/Megarun_2-3/CombinedData_7_1920_22";
             "../Megaruns/Megarun_2-3/CombinedData_7_1920_22";
             "../Megaruns/Megarun_4/CombinedData_7_21_22";
             "../Megaruns/Megarun_5/CombinedData_7_22_22"
             "../Megaruns/Megarun_6/CombinedData_7_25_22"
             "../Megaruns/Megarun_7/CombinedData_7_25_22"];
filePath = filePaths(megarun);

data = load(sprintf("%s.mat", filePath));
fprintf("Loaded data from %s.mat.\n", filePath);

%% Fix duplicate rngSeed
% Sometimes rngSeed is saved twice for each simulation
if(false)
    data.rngSeed(1:2:length(data.rngSeed)) = [];
    fprintf("Removed duplicate rngSeeds.\n");
end

%% Split timeStart and timeEnd
% Sometimes timeStart and timeEnd are concatonated into one massive char
% array, rather than an array of many strings
if(true)
    % timeStart
    if(mod(length(data.timeStart), 8) ~= 0)
        fprintf("TimeStart has bad length. Skipping.\n");
    else
        timeStartOG = data.timeStart;
        data.timeStart = strings(1, 0);

        temp = timeStartOG;
        while(~isempty(temp))
            data.timeStart = [data.timeStart, string(temp(1:8))];
            temp(1:8) = [];
        end
        fprintf("Split timeStart.\n");

        % Double check
        timeStartNew = '';
        for i=1:length(data.timeStart)
            timeStartNew = [timeStartNew, char(data.timeStart(i))]; %#ok<AGROW> 
        end
        eq = unique(timeStartNew == timeStartOG);
        if(isequal(timeStartNew, timeStartOG))
            fprintf("\tNew and old timeStarts are equivalent.\n");
        else
            fprintf("\tWARNING: New and old timeStarts are NOT equivalent.\n");
        end
    end

    % timeEnd
    if(mod(length(data.timeEnd), 8) ~= 0)
        fprintf("TimeEnd has bad length. Skipping.\n");
    else
        timeEndOG = data.timeEnd;
        data.timeEnd = strings(1, 0);

        temp = timeEndOG;
        while(~isempty(temp))
            data.timeEnd = [data.timeEnd, string(temp(1:8))];
            temp(1:8) = [];
        end
        fprintf("Split timeEnd.\n");

        % Double check
        timeEndNew = '';
        for i=1:length(data.timeEnd)
            timeEndNew = [timeEndNew, char(data.timeEnd(i))]; %#ok<AGROW> 
        end
        eq = unique(timeEndNew == timeEndOG);
        if(isequal(timeEndNew, timeEndOG))
            fprintf("\tNew and old timeEnds are equivalent.\n");
        else
            fprintf("\tWARNING: New and old timeEnds are NOT equivalent.\n");
        end
    end
end

%% Save new data
newFilePath = sprintf("%s_New.mat", filePath);
save(newFilePath,'-struct','data');
fprintf("Saved new data to %s.\n", newFilePath);