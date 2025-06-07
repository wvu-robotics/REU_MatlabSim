clc; clear; close all
addpath("Data");
mr7 = load("Megarun7.mat");

filter = mr7.funcName_agentControl == "agentControl_Unified" & mr7.numAgents == 40 & mr7.surviving == 40 & mr7.alignment == 0.001 & mr7.numThermals == 9;
sum(filter)
unique(mr7.numThermals(filter))
sims = find(filter);
sim = sims(2);
describeSim(mr7,sim);

%% Organized Functions
function plotCohesionSeparationSurviving(data,boolFilter)
    figure;
    plot3(data.cohesion(boolFilter),data.separation(boolFilter),data.surviving(boolFilter),'o');
    xlabel("Cohesion");
    ylabel("Separation");
    zlabel("Surviving");
    set(gca,"XScale",'log');
    set(gca,"YScale",'log');

end

function filterSims(data,boolFilter)
    targetSims = find(boolFilter);
    fprintf("Number of Filtered Sims: %g\n\n",length(targetSims));
    targetSim = targetSims(1);
    describeSim(data,targetSim);
end

function describeSim(data,index)
    % Prepare specific variable names
    allvars = ["cohesion","separation","alignment","cohPower","migration","numThermals","numAgents","rngSeed","funcName_agentControl"];
    outputMetrics = ["surviving","collisionDeaths","groundDeaths","flightTime","heightScore","explorationPercent","thermalUseScore","finalHeightMax","finalHeightMin","finalHeightAvg"];
    % List variables for targetSim
    fprintf("Values of Chosen Target Sim @ Index %g\n",index);
    for i=1:length(allvars)
        varData = data.(allvars(i));
        targetVarData = varData(index);
        if(class(targetVarData) == "string" || class(targetVarData) == "char")
            fprintf("%s = %s\n",allvars(i),varData(index));
        else
            fprintf("%s = %g\n",allvars(i),varData(index));
        end
    end
    fprintf("\nOutput Metrics of Chosen Target Sim @ Index %g\n",index);
    for i=1:length(outputMetrics)
        varData = data.(outputMetrics(i));
        targetVarData = varData(index);
        if(class(targetVarData) == "string" || class(targetVarData) == "char")
            fprintf("%s = %s\n",outputMetrics(i),varData(index));
        else
            fprintf("%s = %g\n",outputMetrics(i),varData(index));
        end
    end
end
