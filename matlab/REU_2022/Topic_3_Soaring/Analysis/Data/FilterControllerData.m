clc; clear; close all;

filterTarget = "Baseline";

mr7 = load("Megarun7.mat");
mr7.rngSeed = mr7.rngSeed(1:2:end);
mr7.survivingPercent = mr7.surviving./mr7.numAgents;

indexTarget = mr7.funcName_agentControl == sprintf("agentControl_%s",filterTarget);

names = fieldnames(mr7);
for i=1:length(names)
    allData = mr7.(names{i});
    mr7.(names{i}) = allData(indexTarget);
end

saveName = sprintf("Megarun7%s.mat",filterTarget);
save(saveName,"-struct","mr7");