clear
clc
close all

paramRows = 5;
paramCols = 4;
parameters = optimizableVariable.empty;
for i = 1:paramRows*paramCols
    name = char(sprintf('param%i',i));
    parameters(end + 1) = optimizableVariable(name,[-5,5]);
end

timeSteps = 200;
display = false;
fun = @(param)VectorOptimizationRun(param, display, timeSteps);
results = bayesopt(fun,parameters);