clear
clc
close all

timeSteps = 200;

%load('results/firstResults.m','-mat');
load('results/secondResults.m','-mat');
%load('results/thirdResults.m','-mat');
%load('results/fourthResults.m','-mat');
%load('results/fifthResults.m','-mat');
%load('results/sixthResults.m','-mat');
%load('results/seventhResults.m','-mat');
%load('results/eigthResults.m','-mat');
%load('results/ninthResults.m','-mat');
%load('results/tenthResults.m','-mat');
parameters = results.XAtMinObjective;
cost = VectorOptimizationRun(parameters, true, timeSteps);
for i = 1:size(parameters,2)
    name = char(sprintf('param%i',i));
    newParameters(i) = parameters.(name);
end
newParameters = reshape(newParameters, [7,3]);