% Ind S, C, A, M, hPriority, hIgnore, dt, Waggle Str, Waggle Time, NAgents
ParamMatrix = readmatrix("Params.xlsx","Range","A2:K2");
render  = true;
Write   = false;


average   = zeros(1,size(ParamMatrix,1));
surviving = average;
tic
N = size(ParamMatrix,1);
% parfor if render is false, for if render is true
for i = 1:N
    [average(i), surviving(i)] = MainScriptFunction(ParamMatrix(i,:), render);
end
clf
load gong.mat;
sound(y)
toc
hold on
yyaxis left
plot(average);
yyaxis right
plot(surviving);
legend('Average Height','Number Surviving')
if Write
    writematrix(surviving', "Params.xlsx", 'Range','L2')
    writematrix(average', "Params.xlsx", 'Range','M2')
end