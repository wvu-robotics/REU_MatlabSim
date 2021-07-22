clear
clc
close all

paramRows = 4;
paramCols = 3;
parameters = optimizableVariable.empty;
for i = 1:paramRows*paramCols
    name = char(sprintf('param%i',i));
    parameters(end + 1) = optimizableVariable(name,[-.5,.5]);
end

load('Results/results1.m', '-mat');
goodParameters = results.XAtMinObjective;


display = false;
fun = @(param)SingleAvoidOptimizeable(param, display);
results = bayesopt(fun,parameters, 'MaxObjectiveEvaluations', 100, 'ExplorationRatio', 0.05, 'InitialX', goodParameters);

%{
save('Results/results2.m', '-mat');

load('Results/results1.m', '-mat');
display = true;
parameters = results.XAtMinObjective;
SingleAvoidOptimizeable(parameters, display)
%}
