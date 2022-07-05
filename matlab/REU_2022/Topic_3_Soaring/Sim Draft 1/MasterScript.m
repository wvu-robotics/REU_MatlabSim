% Ind S, C, A, M, hPriority, hIgnore, dt, Waggle Str, Waggle Time, NAgents
clear
clc
ParamMatrix = readmatrix("Params.xlsx","Range","A2:O9");
render  = false;
plotters    = true;
Write   = true;

N = size(ParamMatrix,1);
average   = zeros(1,N);
surviving = average;
Time      = surviving;%             \/ index for N_Agents
ToD       = zeros(max(ParamMatrix(:,11)),N);
Log       = cell(N, 26);
data      = cell(1,N);
tic
% parfor if render is false, for if render is true
for i = 1:N
    [outData, average(i), surviving(i), Time(i), Log(i,:)] = MainScriptFunction(ParamMatrix(i,:), render);
    data(i) = {outData};
end
if render
    close all
end
[y,Fs] = audioread("microwave.mp3");
%load gong.mat;
sound(y,Fs)
toc

%% Plot
if plotters
    clf
    hold on
%     yyaxis left
%     plot(average);
%     yyaxis right
%     plot(surviving);
%     legend('Average Height','Number Surviving')
    for i = 1:N
        figure(i)
        plot(cell2mat(data(i))')
        title(sprintf("Run # %g",i))
    end
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