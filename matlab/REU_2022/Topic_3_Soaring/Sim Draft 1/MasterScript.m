% Ind S, C, A, M, hPriority, hIgnore, dt, Waggle Str, Waggle Time, NAgents
ParamMatrix = readmatrix("Params.xlsx","Range","A2:K25");
render  = false;
plot    = false;
Write   = true;

N = size(ParamMatrix,1);
average   = zeros(1,N);
surviving = average;
Time      = surviving;
ToD       = zeros(max(ParamMatrix(:,11)),N);
Log       = cell(N, 24);
tic

% parfor if render is false, for if render is true
parfor i = 1:N
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
if Write
    writematrix(surviving', "Params.xlsx", 'Range','L2')
    writematrix(average', "Params.xlsx", 'Range','M2')
    writecell(Log,"Params.xlsx","Sheet",sheetName,'WriteMode','append');
end