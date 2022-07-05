% Ind S, C, A, M, hPriority, hIgnore, dt, Waggle Str, Waggle Time, NAgents
clear
clc
ParamMatrix = readmatrix("Params.xlsx","Range","A2:O2");
render  = true;
plot    = false;
Write   = true;

N = size(ParamMatrix,1);
average   = zeros(1,N);
surviving = average;
Time      = surviving;%             \/ index for N_Agents
ToD       = zeros(max(ParamMatrix(:,11)),N);
Log       = cell(N, 26);
tic

% parfor if render is false, for if render is true
for i = 1:N
    [average(i), surviving(i), Time(i), ToD(:,i), Log(i,:)] = MainScriptFunction(ParamMatrix(i,:), render);
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
sheetName = sprintf("Log %02g-%02g", month(today), day(today));
sheetHeader = ["Date","Time","S","C","A","M","HCoh","HSep","Waggle","WaggleTime","K",...
               "dt","TotalTime","# Agents","# Thermals","VisionRadius","FOV","CtrlFunction","NghbrFunction",...
               "MinSpeed","MaxSpeed","ThermStrMin","ThermStrMax","Survivors","AvgHeight","FlightTime"];
if Write
    writematrix(sheetHeader,"Params.xlsx","Sheet",sheetName,'Range','A1');
    writecell(Log,"Params.xlsx","Sheet",sheetName,'WriteMode','append');
end