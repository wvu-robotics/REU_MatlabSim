% Ind S, C, A, M, hPriority, hIgnore, dt, Waggle Str, Waggle Time, NAgents
clear
clc
%ParamMatrix = readmatrix("Params.xlsx","Range","A2:O7");
render  = true;
plot    = false;
Write   = true;

%N = size(ParamMatrix,1); % Number of trials
average   = zeros(1,1); % column is number of trials
surviving = average;
Time      = surviving;   %          \/ index for N_Agents
ToD       = zeros(30,8); %zeros(max(ParamMatrix(:,11)),8); -> rows depends on number of agents
Log       = cell(1, 31); % row is number of trials
tic

% parfor if render is false, for if render is true
for i = 1:1
    [average(i), surviving(i), Time(i), ToD(:,i), Log(i,:)] = ThermalScriptFunction(render);
    %[average(i), surviving(i), Time(i), ToD(:,i), Log(i,:)] = ThermalScriptFunction(ParamMatrix(i,:), render);
end
if render
    close all
end
load gong.mat;
sound(y)
toc

%% Plot
if plot
    clf
    hold on
    yyaxis left
    plot(average);
    yyaxis right
    plot(surviving);
    legend('Average Height','Number Surviving')
end

%% Write
sheetName = sprintf("Log %02g-%02g SB", month(today), day(today));
sheetHeader = ["Date","Time","S","C","A","M","HCoh","HSep","Waggle","WaggleTime","K",...
               "dt","TotalTime","# Agents","# Thermals","VisionRadius","FOV","CtrlFunction","NghbrFunction",...
               "MinSpeed","MaxSpeed","ThermStrMin","ThermStrMax","ThermRadMin","ThermRadMax","FadeRate", "MinPlat", "MaxPlat",...
               "Deaths due to Collisions", "Survivors","AvgHeight","FlightTime"];
if Write
    writematrix(sheetHeader,"Params.xlsx","Sheet",sheetName,'Range','A1');
    writecell(Log,"Params.xlsx","Sheet",sheetName,'WriteMode','append');
end