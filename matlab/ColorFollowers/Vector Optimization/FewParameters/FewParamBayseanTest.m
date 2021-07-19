clear
clc
close all

load('C:\Users\Stephen Jacobs\Documents\GitHub\REU_MatlabSim\matlab\ColorFollowers\Vector Optimization\FewParameters/Results/results2.m','-mat');
%load('secondResults.m','-mat');
%load('thirdResults.m','-mat');
%load('results/fourthResults.m','-mat');
%load('results/fifthResults.m','-mat');
%load('results/sixthResults.m','-mat');
%load('results/seventhResults.m','-mat');
%load('results/eigthResults.m','-mat');
%load('results/ninthResults.m','-mat');
%load('results/tenthResults.m','-mat');
parameters = results.XAtMinObjective;
cost = FewParamOptimizeable(parameters, true);