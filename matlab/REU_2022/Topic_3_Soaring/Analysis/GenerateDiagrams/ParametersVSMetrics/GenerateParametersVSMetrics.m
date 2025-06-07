clc; clear; close all;
saveFigs = true;
saveDPI = 480;
scale = "log";


data = load("..\..\Data\Megarun7Unified.mat");
data.survivingPercent = round(data.survivingPercent*2)/2;
varNames = ["cohesion","separation","alignment","cohPower","migration","numThermals","numAgents","rngSeed"];
metrics = ["survivingPercent","heightScore","explorationPercent","thermalUseScore"];
numBinsPerMetric = [20,40,40,40];
figs = cell(length(varNames),1);

for i=1:length(varNames)
    varName = varNames(i);
    fig = figure('NumberTitle','off','Name',varName,'Units','normalized','OuterPosition',[0,0,1,1]);
    figs{i} = fig;

    uniqueValues = unique(data.(varName));
    numValues = length(uniqueValues);
    numMetrics = length(metrics);
    tiledlayout(numValues,numMetrics);

    for j=1:numMetrics
        metric = metrics(j);
        binMax = max(data.(metric));
        numBins = numBinsPerMetric(j);
        binEdges = 0:binMax/numBins:binMax;

        maxY = 0;
        for k=1:numValues
            nexttile(j + (k-1)*numMetrics);
            uniqueVal = uniqueValues(k);

            metricData = data.(metric);
            tempData = metricData(data.(varName) == uniqueVal);
            histogram(tempData,'BinEdges',binEdges);
            title(sprintf("%s = %g",varName,uniqueVal));
            xlabel(metric);
            ylabel("Number of Sims");
            a = gca;
            a.YScale = scale;
            currY = ylim;
            maxY = max(maxY,currY(2));
        end
        for k=1:numValues
            nexttile(j + (k-1)*numMetrics);
            ylim([1, 1.2*maxY]);
            if(scale == "log")
                ticks = [1, (1.2*maxY)^(1/3), (1.2*maxY)^(2/3), 1.2*maxY];
            elseif(scale == "linear")
                ticks = [1, (1.2*maxY)*(1/3), (1.2*maxY)*(2/3), 1.2*maxY];
            end
            ticks = round(ticks);
            yticks(ticks);
            yticklabels(string(ticks));
        end
    end
    fprintf("Finished %s.\n",varName);
end

if(saveFigs)
    for i=1:length(varNames)
        varName = varNames(i);
        figName = sprintf("Unified_%s_%s.png",scale,varName);
        fprintf("Saving figure: %s... ",figName);
        exportgraphics(figs{i},figName,'Resolution',saveDPI);
        fprintf("Done!\n");
    end
end

for i=3:3:12
    temp = data.survivingPercent(data.numThermals==i & data.survivingPercent > 0);
    fprintf("NumThermals=%g: Mean: %.2f, Std: %.2f\n",i,mean(temp),std(temp));
end