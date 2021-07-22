clear
clc
close all

iterations = 100;
paramRows = 4;
paramCols = 3;
parameters = rand(paramRows, paramCols) - 0.5 * 2;
display = false;
costs = zeros(iterations, 1);
for i = 1:iterations
costs(i) = SingleAvoidOptimizeable(parameters, display);
end
histogram(costs);