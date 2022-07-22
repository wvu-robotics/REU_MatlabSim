%% Prepare workspace
%close all
clear
clc

%% Load data
run = 5;

if(run == 23)
    fileName = "..\Megaruns\Megarun_2-3\7-1920-22\CombinedData_7_1920_22.mat";
    labels = ["rngSeed","cohesion","heightFactorPower","cohesionAscensionIgnore","cohesionAscensionMax","ascensionFactorPower","separation","alignment"];
elseif(run == 4)
    fileName = "..\Megaruns\Megarun_4\7-21-22\CombinedData_7_21_22.mat";
    labels = ["rngSeed","cohesion","cohesionAscensionIgnore","cohPower","separation","alignment","k"];
elseif(run == 5)
    fileName = "..\Megaruns\Megarun_5\7-22-22\CombinedData_7_22_22.mat";
    labels = ["rngSeed","cohesion","cohPower","separation","alignment","separationHeightWidth","alignmentHeightWidth"];
end
data = load(fileName);

%% Prepare data
X = [];
N = [];
for i=1:length(labels)
    X = [X,data.(labels(i))'];
    N = [N,length(unique(data.(labels(i))))];
end
Y = data.surviving';

%% Feature Selection
[idx_chi2,scores_chi2] = fscchi2(X,Y);
[idx_mrmr,scores_mrmr] = fscmrmr(X,Y);
scores_chi2(isinf(scores_chi2)) = 700;
scores_mrmr(isinf(scores_mrmr)) = 700;

%% Display results
catLabels = categorical(labels);
catLabels = reordercats(catLabels,labels);

figure
tiledlayout(1,3);

nexttile;
bar(catLabels,scores_chi2);
title('FSC: chi2');
xlabel('Variables')
ylabel('Predictor importance score')

nexttile;
bar(catLabels,scores_mrmr);
title('FSC: mrmr');
xlabel('Variables')
ylabel('Predictor importance score')

nexttile;
bar(catLabels,N);
title('Number of Values');
xlabel('Variables')
ylabel('Number of values')
