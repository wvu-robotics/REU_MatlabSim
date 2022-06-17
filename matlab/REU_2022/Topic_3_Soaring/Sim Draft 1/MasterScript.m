% Ind S, C, A, M, hPriority, hIgnore
ParamMatrix = readmatrix("Params.xlsx","Range","A2:G31");
render = false;
average = zeros(1,size(ParamMatrix,1));
surviving = average;
tic
for i = 1:size(ParamMatrix,1)
    ParamS = 10^ParamMatrix(i,2);
    ParamC = 10^ParamMatrix(i,3);
    ParamA = 10^ParamMatrix(i,4);
    ParamM = 10^ParamMatrix(i,5);
    ParamhPr = ParamMatrix(i,6);
    ParamhIg = ParamMatrix(i,7);
    [average(i), surviving(i)] = MainScriptFunction(ParamS, ParamC, ParamA, ParamM, ParamhPr, ParamhIg, i, render);
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