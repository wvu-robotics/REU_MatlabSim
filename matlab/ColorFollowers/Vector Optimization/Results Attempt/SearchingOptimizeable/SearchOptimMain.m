clear
clc
close all

paramRows = 5;
paramCols = 4;
parameters = optimizableVariable.empty;
for i = 1:paramRows*paramCols
    name = char(sprintf('param%i',i));
    parameters(end + 1) = optimizableVariable(name,[-4,4]);
end

load('results7.m', '-mat');
goodParameters = results.XAtMinObjective;

display = false;
fun = @(param)SearchingOptimScript(param, display);
results = bayesopt(fun,parameters);

%{
save('results8.m', '-mat');

close all
load('results8.m', '-mat');
display = true;
parameters = results.XAtMinObjective;
SearchingOptimScript(parameters, display)
%}