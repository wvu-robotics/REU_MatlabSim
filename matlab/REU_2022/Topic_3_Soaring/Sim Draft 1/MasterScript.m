% Ind S, C, A, M, hPriority, hIgnore, dt, Waggle Str, Waggle Time, NAgents
ParamMatrix = readmatrix("Params.xlsx","Range","A2:K7");
render = true;
average = zeros(1,size(ParamMatrix,1));
surviving = average;
tic
for i = 1:size(ParamMatrix,1)
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

writematrix(surviving', "Params.xlsx", 'Range','L2')
writematrix(average', "Params.xlsx", 'Range','M2')