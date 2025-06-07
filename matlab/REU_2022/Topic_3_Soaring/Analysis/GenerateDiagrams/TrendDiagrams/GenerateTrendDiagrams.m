clc; clear; close all;
saveFigs = true;
saveDPI = 1500; %480 
%fontSize = 30;

data = load("..\..\Data\Megarun7Unified.mat");
data.survivingPercent = round(data.survivingPercent*2)/2;
varNames = ["cohesion","separation","alignment","cohPower","migration","numThermals","numAgents","rngSeed"];
metrics = ["survivingPercent","heightScore","explorationPercent","thermalUseScore"];
numBinsPerMetric = [20,40,40,40];

%Trend for number of agents
%multiFig_numAgents = drawNumAgents(data, "numAgents", "log", saveFigs, saveDPI, 20); %40

%Trend for number of thermals
%multiFig_numThermals = drawNumThermals(data, "numThermals", "log", saveFigs, saveDPI, 20); %40

%Trend for migration
%multiFig_migration = drawMigration(data, "migration", "log", saveFigs, saveDPI, 12); %33

%{
migrationScale = "log";
migrationSpec = cell(3,3);
migrationSpec{1,1} = ["migration","1e-23","survivingPercent","20",migrationScale];
migrationSpec{2,1} = ["migration","1e-22","survivingPercent","20",migrationScale];
migrationSpec{3,1} = ["migration","1e-21","survivingPercent","20",migrationScale];
migrationSpec{1,2} = ["migration","1e-23","explorationPercent","40",migrationScale];
migrationSpec{2,2} = ["migration","1e-22","explorationPercent","40",migrationScale];
migrationSpec{3,2} = ["migration","1e-21","explorationPercent","40",migrationScale];
migrationSpec{1,3} = ["migration","1e-23","thermalUseScore","40",migrationScale];
migrationSpec{2,3} = ["migration","1e-22","thermalUseScore","40",migrationScale];
migrationSpec{3,3} = ["migration","1e-21","thermalUseScore","40",migrationScale];
multiFig_migration = drawMultiFigure(migrationSpec, "migration", data, saveFigs, saveDPI, fontSize);
%}

%Trend for cohesion
cohesionScale = "log";
cohesionSpec = cell(2,2);
cohesionSpec{1,1} = ["cohesion","1000","survivingPercent","20",cohesionScale];
cohesionSpec{2,1} = ["cohesion","100000","survivingPercent","20",cohesionScale];
cohesionSpec{1,2} = ["cohesion","1000","thermalUseScore","40",cohesionScale];
cohesionSpec{2,2} = ["cohesion","100000","thermalUseScore","40",cohesionScale];
%multiFig_cohesion = drawMultiFigure(cohesionSpec, "cohesion", data, saveFigs, saveDPI, fontSize);

%Cohesion and Separation

%cohsepMultiFig = drawCohesionSeparationMultiFigure(data, "survivingPercent", "log", 20, saveFigs, saveDPI, 10);
cohSepSurfFig = drawCohesionSeparationSurfFigure(data,"cohSepSurf",saveFigs,saveDPI, 20); %40

%% Helper Functions
function multiFig = drawMultiFigure(multiFigSpec, multiFigName, data, saveFigs, saveDPI, fontSize)
    multiFigSize = size(multiFigSpec);
    multiFig = figure('NumberTitle','off','Name',multiFigName,'Units','normalized','OuterPosition',[0,0,1,1]);
    tiledlayout(multiFigSize(1),multiFigSize(2));

    maxYMap = containers.Map('KeyType','char','ValueType','double');

    for i=1:multiFigSize(1)
        for j=1:multiFigSize(2)
            figInfo = multiFigSpec{i,j};

            varName = figInfo(1);
            varValue = double(figInfo(2));
            metricName = figInfo(3);
            numBins = double(figInfo(4));
            figScale = figInfo(5);

            nexttile(j + (i-1)*multiFigSize(2));
        
            binMax = max(data.(metricName));
            binEdges = 0:binMax/numBins:binMax;

            metricData = data.(metricName);
            tempData = metricData(data.(varName) == varValue);
            histogram(tempData,'BinEdges',binEdges);
            title(sprintf("%s = %g",varName,varValue));
            xlabel(metricName);
            ylabel("Number of Sims");
            a = gca;
            a.YScale = figScale;
            a.FontSize = fontSize;

            currY = ylim;
            currY = currY(2);
            maxYMapKey = sprintf("%s_%s",varName,metricName);
            if(~isKey(maxYMap,maxYMapKey))
                maxYMap(maxYMapKey) = currY;
            else
                maxYMap(maxYMapKey) = max(currY,maxYMap(maxYMapKey));
            end
        end
    end
    for i=1:multiFigSize(1)
        for j=1:multiFigSize(2)
            figInfo = multiFigSpec{i,j};

            varName = figInfo(1);
            metricName = figInfo(3);
            figScale = figInfo(5);

            nexttile(j + (i-1)*multiFigSize(2));
            maxYMapKey = sprintf("%s_%s",varName,metricName);
            maxY = maxYMap(maxYMapKey);
            maxYTick = 1.2*maxY;
            ylim([1, maxYTick]);

            if(figScale == "log")
                if(maxYTick < 10)
                    ticks = [1, maxYTick];
                elseif(maxYTick < 500)
                    ticks = [1, 10, maxYTick];
                elseif(maxYTick < 5000)
                    ticks = [1, 10, 100, maxYTick];
                elseif(maxYTick < 50000)
                    ticks = [1, 10, 100, 1000, maxYTick];
                else
                    ticks = [1, 100, 1000, 10000, maxYTick];
                end
            elseif(figScale == "linear")
                ticks = [1, maxYTick*(1/3), maxYTick*(2/3), maxYTick];
            end
            ticks = round(ticks);
            yticks(ticks);
            yticklabels(string(ticks));
        end
    end
    if(saveFigs)
        figName = sprintf("Trend_%s.png",multiFigName);
        fprintf("Saving figure: %s... ",figName);
        exportgraphics(multiFig,figName,'Resolution',saveDPI);
        fprintf("Done!\n");
    end
end

function multiFig = drawNumAgents(data, figName, yscale, saveFigs, saveDPI, fontSize)
    data20 = data.survivingPercent(data.numAgents == 20);
    data40 = data.survivingPercent(data.numAgents == 40);

    multiFig = figure('NumberTitle','off','Name',figName);
    multiFig.Units = "pixels";
    multiFig.Position = [0,0,1300,700];
    binEdges = 0:5:100;
    N20 = histcounts(data20,binEdges);
    N40 = histcounts(data40,binEdges);
    N20(length(N20)+1)=N20(length(N20));
    N40(length(N40)+1)=N40(length(N40));
    hold on
    lineWidth = 2.5; %3
    stairs(binEdges,N20,'LineWidth',lineWidth,'LineStyle','-');
    stairs(binEdges,N40,'LineWidth',lineWidth,'LineStyle','--');
    hold off
    a = gca;
    a.YScale = yscale;
    a.FontSize = fontSize;
    
    xlabel("% of Surviving Agents");
    ylabel("Number of Simulations");
    ylim([10,10000]);
    leg = legend(["Number of Agents = 20","Number of Agents = 40"],'Location','northeast');
    leg.FontSize = 24;
    grid on
    
    if(saveFigs)
        figSaveName = sprintf("Trend_%s.png",figName);
        fprintf("Saving figure: %s... ",figSaveName);
        exportgraphics(multiFig,figSaveName,'Resolution',saveDPI);
        fprintf("Done!\n");
    end
end

function multiFig = drawNumThermals(data, figName, yscale, saveFigs, saveDPI, fontSize)
    multiFig = figure('NumberTitle','off','Name',figName);
    multiFig.Units = "pixels";
    multiFig.Position = [0,0,1300,700];
    unThermals = unique(data.numThermals);
    legendVals = strings(1,0);
    hold on
    lineStyles = ["-","--",":","-."];
    for i=1:length(unThermals)
        val = unThermals(i);
        tempData = data.survivingPercent(data.numThermals == val);
        binEdges = 0:5:100;
        N = histcounts(tempData, binEdges);
        N(length(N)+1)=N(length(N));
        lineWidth = 2.5; %3
        stairs(binEdges,N,'LineWidth',lineWidth,'LineStyle',lineStyles(i));
        legendVals(i) = sprintf("Number of Thermals = %g",val);
    end
    hold off
    a = gca;
    a.YScale = yscale;
    a.FontSize = fontSize;
    xlabel("% of Surviving Agents");
    ylabel("Number of Simulations");
    ylim([10,10000]);
    leg = legend(legendVals,'Location','northeast');
    leg.FontSize = 24;
    grid on
    
    if(saveFigs)
        figSaveName = sprintf("Trend_%s.png",figName);
        fprintf("Saving figure: %s... ",figSaveName);
        exportgraphics(multiFig,figSaveName,'Resolution',saveDPI);
        fprintf("Done!\n");
    end
end

function multiFig = drawMigration(data, figName, yscale, saveFigs, saveDPI, fontSize)
    multiFig = figure('NumberTitle','off','Name',figName);
    multiFig.Units = "pixels";
    multiFig.Position = [0,0,1050,280];
    tiledlayout(1,3);
    unMigration = unique(data.migration);
    legendVals = strings(1,0);
    colors = colormap(turbo);
    colors = colors([1,100,200],:);
    lineWidth = 1.5;
    for i=1:length(unMigration)
        val = unMigration(i);
        legendVals(i) = sprintf("Migration = %g",val);
        
        nexttile(1);
        dataSurv = data.survivingPercent(data.migration == val);
        survBinEdges = 0:5:100;
        N = histcounts(dataSurv, survBinEdges);
        N(length(N)+1)=N(length(N));
        hold on
        stairs(survBinEdges,N,'LineWidth',lineWidth,'Color',colors(i,:));
        hold off
        
        nexttile(2);
        dataExplore = data.explorationPercent(data.migration == val);
        exploreBinEdges = 0:2.5:100;
        N = histcounts(dataExplore, exploreBinEdges);
        N(length(N)+1)=N(length(N));
        hold on
        stairs(exploreBinEdges,N,'LineWidth',lineWidth,'Color',colors(i,:));
        hold off
        
        nexttile(3);
        dataThermalUse = data.thermalUseScore(data.migration == val);
        maxBin = 100000;
        thermalUseBinEdges = 0:maxBin/40:maxBin;
        N = histcounts(dataThermalUse, thermalUseBinEdges);
        N(length(N)+1)=N(length(N));
        hold on
        stairs(thermalUseBinEdges,N,'LineWidth',lineWidth,'Color',colors(i,:));
        hold off
    end
    
    nexttile(1);
    a = gca;
    a.YScale = yscale;
    a.FontSize = fontSize;
    xlabel("Percent of Surviving Agents");
    ylabel("Number of Simulations");
    ticks = [1, 10, 100, 1000, 10000];
    yticks(ticks);
    yticklabels(["1","10","100","1000","10,000"]);
    
    nexttile(2);
    a = gca;
    a.YScale = yscale;
    a.FontSize = fontSize;
    xlabel("Exploration Percent");
    ylabel("Number of Simulations");
    ticks = [1, 10, 100, 1000, 10000];
    yticks(ticks);
    yticklabels(["1","10","100","1000","10,000"]);
    
    nexttile(3);
    a = gca;
    a.YScale = yscale;
    a.FontSize = fontSize;
    xlabel("Thermal Use Score");
    ylabel("Number of Simulations");
    ticks = [1, 10, 100, 1000, 10000];
    yticks(ticks);
    yticklabels(["1","10","100","1000","10,000"]);
    a.XAxis.ExponentMode = 'manual';
    a.XAxis.Exponent = 0;
    xticklabels(["0","50,000","100,000"]);
    
    leg = legend(legendVals,'Location','northeast');
    leg.ItemTokenSize = [10;18];
    
    if(saveFigs)
        figName = sprintf("Trend_%s.png",figName);
        fprintf("Saving figure: %s... ",figName);
        exportgraphics(multiFig,figName,'Resolution',saveDPI);
        fprintf("Done!\n");
    end
end

function multiFig = drawCohesionSeparationMultiFigure(data, metricName, figScale, numBins, saveFigs, saveDPI, fontSize)
    multiFigName = sprintf("CohesionSeparationVS%s",metricName);
    multiFig = figure('NumberTitle','off','Name',multiFigName,'Units','normalized','OuterPosition',[0,0,1,1]);

    un_cohesion = unique(data.cohesion);
    un_separation = unique(data.separation);
    num_cohesion = length(un_cohesion);
    num_separation = length(un_separation);

    tiledlayout(num_cohesion,num_separation);

    maxY = 0;
    for i=1:num_cohesion
        for j=1:num_separation
            value_cohesion = un_cohesion(i);
            value_separation = un_separation(j);

            nexttile(j+(i-1)*num_separation);
            tempData = data.(metricName);
            tempData = tempData(data.cohesion==value_cohesion & data.separation==value_separation);

            binMax = max(data.(metricName));
            binEdges = 0:binMax/numBins:binMax;
            histogram(tempData,'BinEdges',binEdges);
            title(sprintf("cohesion = %.0e\nseparation = %.0e\nmean=%.2f",value_cohesion,value_separation,mean(tempData)));
            xlabel(metricName);
            ylabel("Number of Sims");
            a = gca;
            a.YScale = figScale;
            a.FontSize = fontSize;

            currY = ylim;
            maxY = max(maxY,currY(2));
        end
    end
    for i=1:num_cohesion
        for j=1:num_separation
            nexttile(j+(i-1)*num_separation);
            maxYTick = 1.2*maxY;
            ylim([1, maxYTick]);
            if(figScale == "log")
                if(maxYTick < 10)
                    ticks = [1, maxYTick];
                elseif(maxYTick < 500)
                    ticks = [1, 10, maxYTick];
                elseif(maxYTick < 5000)
                    ticks = [1, 10, 100, maxYTick];
                elseif(maxYTick < 50000)
                    ticks = [1, 10, 100, 1000, maxYTick];
                else
                    ticks = [1, 100, 1000, 10000, maxYTick];
                end
            elseif(figScale == "linear")
                ticks = [1, maxYTick*(1/3), maxYTick*(2/3), maxYTick];
            end
            ticks = round(ticks);
            yticks(ticks);
            yticklabels(string(ticks));
        end
    end
    if(saveFigs)
        fileName = sprintf("%s.png",multiFigName);
        fprintf("Saving figure: %s... ",fileName);
        exportgraphics(multiFig,fileName,'Resolution',saveDPI);
        fprintf("Done!\n");
    end
end

function multiFig = drawCohesionSeparationSurfFigure(data,figName,saveFigs,saveDPI,fontSize)
    unique_sep = unique(data.separation);
    unique_coh = unique(data.cohesion);
    numSep = length(unique_sep);
    numCoh = length(unique_coh);
    values_sep = zeros(numSep,numCoh);
    values_coh = zeros(numSep,numCoh);
    values_surv = zeros(numSep,numCoh);
    for i=1:length(unique_sep)
        for j=1:length(unique_coh)
            sepVal = unique_sep(i);
            cohVal = unique_coh(j);
            tempData = data.survivingPercent(data.separation == sepVal & data.cohesion == cohVal);
            survVal = mean(tempData);
            values_sep(i,j) = sepVal;
            values_coh(i,j) = cohVal;
            values_surv(i,j) = survVal;
        end
    end
    multiFig = figure('NumberTitle','off','Name',figName);
    multiFig.Units = "pixels";
    multiFig.Position = [0,0,1200,700];
    surf(values_sep,values_coh,values_surv,'FaceAlpha',1.0);
    xlabel("Separation");
    ylabel("Cohesion");
    zlabel("Percent of Surviving Agents");
    a = gca;
    a.XScale = "log";
    a.YScale = "log";
    a.FontSize = fontSize;
    a.XTick = unique_sep;
    a.XTickLabels = {a.XTick};
    a.YTick = unique_coh;
    %a.YTickLabels = {a.YTick};
    a.YTickLabels = ["1000","3000","10,000","30,000","100,000"];
    pause(0.001); % Annoying load bearing pause for adjusting ZAxis Label position
    multiFig.Children.ZAxis.Label.Position(3) = 24;
    zlim([0,40]);
    view(-45,30);

    if(saveFigs)
        figName = sprintf("Trend_%s.png",figName);
        fprintf("Saving figure: %s... ",figName);
        exportgraphics(multiFig,figName,'Resolution',saveDPI);
        fprintf("Done!\n");
    end
end