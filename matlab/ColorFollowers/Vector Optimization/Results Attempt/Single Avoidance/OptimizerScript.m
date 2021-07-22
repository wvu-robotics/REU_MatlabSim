clear
clc
close all

paramRows = 4;
paramCols = 3;
parameters = optimizableVariable.empty;
for i = 1:paramRows*paramCols
    name = char(sprintf('param%i',i));
    parameters(end + 1) = optimizableVariable(name,[-.8,.8]);
end

display = false;
fun = @(param)SingleAvoidOptimizeable(param, display);
results = bayesopt(fun,parameters);

%{
display = true;
load('Results/results1.m', '-mat');
parameters = results.XAtMinObjective;
SingleAvoidOptimizeable(parameters, display)
%}
