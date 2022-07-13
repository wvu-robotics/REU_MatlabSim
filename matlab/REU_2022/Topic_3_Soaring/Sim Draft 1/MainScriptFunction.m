function outputData = MainScriptFunction(SL, simNumber, videoName)
    %% Clear
    close all
    %clear
    %clc
    
    %% Set RNG Seed
    if(isfield(SL,"rngSeed") && ~isnan(SL.rngSeed))
        rnd(SL.rngSeed);
    end

    %% Load simulation parameters
    % Initialize thermals as a matrix of Thermals

    thermalMap = ThermalMap(SL);
    swarm = Swarm(SL, thermalMap);

    %% Setup video and figure
    if SL.render
        %videoPrefix = sprintf('[dt %g, T %g, x%g] ',simLaw.dt, simLaw.totalTime, simLaw.fpsMult);
        %videoSuffix = sprintf('%02g',Param(1));
        %videoSuffix = sprintf('%1.0E, %1.0E, %1.0E', simLaw.separation, simLaw.cohesion, simLaw.alignment);
        swarm.initVideo(videoName);
    end

    %% Run simulation...
    steps  = SL.totalTime/SL.dt;
    outputData.heightData = zeros(3,steps);
    outputData.xData = zeros(SL.numAgents,steps);
    outputData.yData = zeros(SL.numAgents,steps);
    outputData.zData = zeros(SL.numAgents,steps);
    outputData.headingData = zeros(SL.numAgents,steps);
    outputData.bankAngleData = zeros(SL.numAgents,steps);
    outputData.fVelData = zeros(SL.numAgents,steps);
    outputData.tVelData = zeros(SL.numAgents,steps);
    outputData.zVelData = zeros(SL.numAgents,steps);
    outputData.timeStart = datestr(now,"HH:MM:SS");

    for step = 1:steps
        if ~SL.stopWhenDead || step == 1 || swarm.Living > 0
            %% Step simulation
            thermalMap.staticStep();
            swarm.saveAgentData();
            swarm.stepSimulation(step);

            %% Read Data
            swarm.updateData(step);
            outputData.heightData(1,step) = swarm.maxHeight;
            outputData.heightData(2,step) = swarm.minHeight;
            outputData.heightData(3,step) = swarm.avgHeight;
            for Agent = 1:SL.numAgents
                outputData.xData(Agent,step) = swarm.agents(Agent).position(1);
                outputData.yData(Agent,step) = swarm.agents(Agent).position(2);
                outputData.zData(Agent,step) = swarm.agents(Agent).position(3);
                outputData.headingData(Agent,step) = swarm.agents(Agent).heading;
                outputData.bankAngleData(Agent,step) = swarm.agents(Agent).bankAngle;
                outputData.fVelData(Agent,step) = swarm.agents(Agent).velocity(1);
                outputData.tVelData(Agent,step) = swarm.agents(Agent).velocity(2);
                outputData.zVelData(Agent,step) = swarm.agents(Agent).velocity(3);
            end

            %% Render
            if mod(step,SL.frameSkip)==0 && SL.render
                swarm.renderAll();
            end

            %% Print
            if mod(step,steps/100) == 0
                fprintf("%02g%% through Run #  %g \n",100*step/steps, simNumber);
            end
        else
            fprintf("Everybody died in Run # %g\n", simNumber);
            break
        end
    end
    if SL.render 
        swarm.closeVideo(); 
    end

    %% Save more output data
    outputData.simNumber = simNumber;
    outputData.timeEnd = datestr(now,"HH:MM:SS");
    outputData.surviving = swarm.Living;
    outputData.flightTime = swarm.flightTime;
    outputData.finalHeightMax = outputData.heightData(1,steps);
    outputData.finalHeightMin = outputData.heightData(2,steps);
    outputData.finalHeightAvg = outputData.heightData(3,steps);
end