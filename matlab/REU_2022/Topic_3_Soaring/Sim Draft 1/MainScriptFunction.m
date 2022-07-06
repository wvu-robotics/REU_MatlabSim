function outputData = MainScriptFunction(SL, simNumber, videoName, render)
    %% Clear
    close all
    %clear
    %clc

    %% Load simulation parameters
    % Initialize thermals as a matrix of Thermals
    thermalMap = ThermalMap(SL);
    swarm = Swarm(SL, thermalMap);

    %% Setup video and figure
    if render
        %videoPrefix = sprintf('[dt %g, T %g, x%g] ',simLaw.dt, simLaw.totalTime, simLaw.fpsMult);
        %videoSuffix = sprintf('%02g',Param(1));
        %videoSuffix = sprintf('%1.0E, %1.0E, %1.0E', simLaw.separation, simLaw.cohesion, simLaw.alignment);
        swarm.initVideo(videoName);
    end

    %% Run simulation...
    steps  = SL.totalTime/SL.dt;
    outputData.heightData = zeros(3,steps);
    outputData.timeStart = datestr(now,"HH:MM:SS");

    for step = 1:steps
        if ~SL.stopWhenDead || step == 1 || swarm.Living > 0
            %% Step simulation
            thermalMap.staticStep();
            swarm.saveAgentData();
            swarm.stepSimulation();

            %% Read Data
            swarm.updateData(step);
            outputData.heightData(1,step) = swarm.maxHeight;
            outputData.heightData(2,step) = swarm.minHeight;
            outputData.heightData(3,step) = swarm.avgHeight;
            
            if mod(step,SL.frameSkip)==0
                %% Render
                if render
                    swarm.renderAll();
                end
            end

            %% Print
            if mod(step,steps/10) == 0
                fprintf("%02g%% through Run #  %g \n",100*step/steps, simNumber);
            end
        else
            fprintf("Everybody died in Run # %g\n", simNumber);
            break
        end
    end
    if render 
        swarm.closeVideo(); 
    end

    %% Save more output data
    outputData.timeEnd = datestr(now,"HH:MM:SS");
    outputData.surviving = swarm.Living;
    outputData.flightTime = swarm.flightTime;
    outputData.finalHeightMax = outputData.heightData(1,steps);
    outputData.finalHeightMin = outputData.heightData(2,steps);
    outputData.finalHeightAvg = outputData.heightData(3,steps);
end