%% Prepare workspace
%close all
clear
clc

%% Load data
fileName = "E:\WVU_REU\7-20-22\CombinedData_7_1920_22.mat";
data = load(fileName);

%% Prepare data
X = [data.rngSeed',data.cohesion',data.heightFactorPower',data.cohesionAscensionIgnore',data.cohesionAscensionMax',data.ascensionFactorPower',data.separation',data.alignment'];
Y = data.surviving';
N = [5,4,3,4,2,3,4,6];

%% Feature Selection
[idx_chi2,scores_chi2] = fscchi2(X,Y);
[idx_mrmr,scores_mrmr] = fscmrmr(X,Y);
scores_chi2(isinf(scores_chi2)) = 500;
scores_mrmr(isinf(scores_mrmr)) = 500;

%% Display results
labels = {'rngSeed','cohesion','heightFactorPower','cohesionAscensionIgnore','cohesionAscensionMax','ascensionFactorPower','Separation','Alignment'};
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
