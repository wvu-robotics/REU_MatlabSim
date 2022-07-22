%% Prepare workspace
%close all
clear
clc

%% Load data
folderName = "E:\WVU_REU\7-21-22\Megarun 4";
fileName = folderName + "\CombinedData_7_21_22.mat";
data = load(fileName);

%% Prepare data
%X = [data.rngSeed',data.cohesion',data.heightFactorPower',data.cohesionAscensionIgnore',data.cohesionAscensionMax',data.ascensionFactorPower',data.separation',data.alignment'];
%N = [5,4,3,4,2,3,4,6];

X = [data.rngSeed',data.cohesion',data.cohesionAscensionIgnore',data.cohPower',data.separation',data.alignment',data.k'];
N = [5,4,8,4,4,4,4];
Y = data.surviving';

%% Feature Selection
[idx_chi2,scores_chi2] = fscchi2(X,Y);
[idx_mrmr,scores_mrmr] = fscmrmr(X,Y);
scores_chi2(isinf(scores_chi2)) = 700;
scores_mrmr(isinf(scores_mrmr)) = 700;

%% Display results
%labels = {'rngSeed','cohesion','heightFactorPower','cohesionAscensionIgnore','cohesionAscensionMax','ascensionFactorPower','Separation','Alignment'};
labels = {'rngSeed','cohesion','cohesionAscensionIgnore','cohPower','separation','alignment','k'};
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
