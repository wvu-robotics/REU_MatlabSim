% Main script: loads parameter variables and runs swarm step function
%% Clear
close all
clear
clc

%% setup output folder
rootFolder = "Output Media";
dateFormat = "mm-dd-yy";
timeFormat = "HH:MM:SS AM";

date = datestr(now,dateFormat);
time = datestr(now,timeFormat);

dateFolder = sprintf("%s\\%s",rootFolder,date);

if(~exist(rootFolder,'dir'))
    mkdir(rootFolder);
end

if(~exist(dateFolder,'dir'))
    mkdir(dateFolder);
end

