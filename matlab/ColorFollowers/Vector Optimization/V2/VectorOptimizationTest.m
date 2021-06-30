clear
clc
close all

paramRows = 5;
paramCols = 4;
parameters = (rand([5,4])-0.5)*10;

cost = VectorOptimizationRun(parameters);