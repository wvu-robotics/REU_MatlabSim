clc; clear; close all
addpath("Megarun Data");
mr7 = load("Megarun7.mat");
allvars = ["cohesion","separation","alignment","cohPower","migration","numThermals","numAgents","rngSeed","funcName_agentControl"];
ucoh = unique(mr7.cohesion);
usep = unqiue(mr7.separation);
ualign = unique(mr7.alignment);
ucohPow = unique(mr7.cohPower);
umig = unique(mr7.migration);