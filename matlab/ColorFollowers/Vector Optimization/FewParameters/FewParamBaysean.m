clear
clc
close all

paramRows = 6;
paramCols = 1;
parameters = optimizableVariable.empty;
for i = 1:paramRows*paramCols
    name = char(sprintf('param%i',i));
    parameters(end + 1) = optimizableVariable(name,[-.5,.5]);
end

display = false;
fun = @(param)FewParamOptimizeable(param, display);
results = bayesopt(fun,parameters);