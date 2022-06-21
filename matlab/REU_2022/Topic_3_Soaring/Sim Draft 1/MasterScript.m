% Ind S, C, A, M, hPriority, hIgnore, dt
ParamMatrix = readmatrix("Params.xlsx","Range","A2:H11");
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
    dt = ParamMatrix(i,8);
    [average(i), surviving(i)] = MainScriptFunction(ParamS, ParamC, ParamA, ParamM, ParamhPr, ParamhIg, dt, i, render);
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

writematrix(surviving', "Params.xlsx", 'Range','I2')
writematrix(average', "Params.xlsx", 'Range','J2')