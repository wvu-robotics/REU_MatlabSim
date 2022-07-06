% Swarm class
classdef Swarm < handle
    properties
        agents
        simLaw
        funcHandle_agentControl
        funcHandle_findNeighborhood
        thisAgent = 0;
        thermalMap
        simFig
        video
        
        lineCircle = NaN
        lineNeighbors = NaN
        lineRange = NaN
        patchSep = NaN
        patchCoh = NaN
        patchAli = NaN
        patchMig = NaN
        patchWag = NaN

        textAnnt = NaN
        number  = 0                 % 
        Elapsed = [0.0 0.0 0.0]     % hours, minutes, seconds
        flightTime = 0.0            % Agent-Seconds
        Living = 0                  %
        avgHeight = 0           % m
        minHeight = 0               % m
        maxHeight = 0               % m
        avgSpeed  = 0               % m/s
    end
    
    methods
        % Generation Function
        function obj = Swarm(simLaw,thermalMap)
            % Save parameters and agentControlFunc
            obj.simLaw = simLaw;
            obj.thermalMap = thermalMap;
            SL = obj.simLaw;
            
            obj.funcHandle_agentControl = str2func(SL.funcName_agentControl);
            obj.funcHandle_findNeighborhood = str2func(SL.funcName_findNeighborhood);
            
            %% Generate agents
            numAgents = SL.numAgents;   %Get total number of agents
            obj.agents = Agent.empty(0,SL.numAgents);
            obj.agents(1,numAgents) = Agent();  %Fill agents with default constructors of Agent
            
            %Iterate through all agents
            posRange = SL.agentSpawnPosRange;
            velRange = SL.agentSpawnVelRange;
            altiRange = SL.agentSpawnAltiRange;
            for i=1:numAgents
                obj.agents(i).simLaw = SL;
                %Set agent initial position, heading, bank angle, velocity
                obj.agents(i).position(1) = Utility.randIR(posRange(1,1),posRange(2,1));
                obj.agents(i).position(2) = Utility.randIR(posRange(1,2),posRange(2,2));
                obj.agents(i).position(3) = Utility.randIR(altiRange(1),altiRange(2));
                obj.agents(i).heading = Utility.randIR(0,2*pi); %rad
                obj.agents(i).bankAngle = 0; %rad
                obj.agents(i).velocity(1) = Utility.randIR(velRange(1,1),velRange(2,1));
                obj.agents(i).velocity(2) = Utility.randIR(velRange(1,2),velRange(2,2));
            end
        end
        
        % Save Function
        function obj = saveAgentData(obj)
            SL = obj.simLaw;
            numAgents = SL.numAgents;
            for i=1:numAgents
                obj.agents(i).saveData();
            end
        end
        
        % Step Function
        function obj = stepSimulation(obj)
            SL = obj.simLaw;
            numAgents = SL.numAgents;   %Get total number of agents
            for i=1:numAgents
                if obj.agents(i).isAlive
                    currentAgent = obj.agents(i);
                    
                    %Find localAgents
                    localAgents = obj.funcHandle_findNeighborhood(obj,i,SL);
                    
                    %Find thermal strength from ThermalMap
                    thermalStrength = obj.thermalMap.getStrength(currentAgent.position);
                    
                    %Update currentAgent
                    obj.funcHandle_agentControl(currentAgent,localAgents,thermalStrength,[0,0,0], SL);
                end
            end
        end
        
        % Render
        function obj = renderAgents(obj)
            SL = obj.simLaw;
            shownNeighbors = false;
            for i=1:SL.numAgents
                if(~shownNeighbors && (SL.showFixedRadius || SL.showNeighbors || SL.showRange || SL.showArrow || SL.showText) && obj.agents(i).isAlive)
                    shownNeighbors = true;
                    currentAgent = obj.agents(i);
                    obj.thisAgent = i;
                    localAgents = obj.funcHandle_findNeighborhood(obj,i,SL);
                    
                    %% Show Vision Radius
                    if(SL.showFixedRadius)
                        theta = linspace(currentAgent.heading-SL.fov/2,currentAgent.heading+SL.fov/2,20);
                        xCircle = SL.neighborRadius * cos(theta) + currentAgent.position(1);
                        yCircle = SL.neighborRadius * sin(theta) + currentAgent.position(2);

                        if(class(obj.lineCircle) == "double")
                            obj.lineCircle = line();
                        end
                        obj.lineCircle.XData = xCircle;
                        obj.lineCircle.YData = yCircle;
                        if ~currentAgent.isAlive
                            obj.lineCircle.Visible = 'off';
                        end
                    end

                    %% Show Lines to Neighbors
                    if(SL.showNeighbors)
                        numLocalAgents = size(localAgents,2);
                        linePoints = zeros(2,2*numLocalAgents+1);
                        linePoints(1,1) = currentAgent.position(1);
                        linePoints(2,1) = currentAgent.position(2);
                        for j=1:numLocalAgents
                            linePoints(1,2*j) = localAgents(j).position(1);
                            linePoints(2,2*j) = localAgents(j).position(2);
                            linePoints(1,2*j+1) = currentAgent.position(1);
                            linePoints(2,2*j+1) = currentAgent.position(2);
                        end

                        if(class(obj.lineNeighbors) == "double")
                            obj.lineNeighbors = line();
                        end
                        obj.lineNeighbors.XData = linePoints(1,:);
                        obj.lineNeighbors.YData = linePoints(2,:);
                    end
                    
                    %% Show Flight Range
                    if(SL.showRange)
                        theta = linspace(0,2*pi,30);
                        xCircleRange = currentAgent.velocity(1)/currentAgent.vsink * currentAgent.position(3) * cos(theta) + currentAgent.position(1);
                        yCircleRange = currentAgent.velocity(1)/currentAgent.vsink * currentAgent.position(3) * sin(theta) + currentAgent.position(2);

                        if(class(obj.lineRange) == "double")
                            obj.lineRange = line();
                        end
                        if currentAgent.position(3) < 200 && currentAgent.isAlive
                            obj.lineRange.XData = xCircleRange;
                            obj.lineRange.YData = yCircleRange;
                            obj.lineRange.Color = [1,0,0];
                            obj.lineRange.Visible = 'on';
                        else
                            obj.lineRange.Visible = 'off';
                        end

                    end
                    
                    %% Show Acceleration Arrows...
                    if(SL.showArrow)
                        %% Calc Arrows
                        SepMatrix      = [cos(currentAgent.rulesDir(1)), -sin(currentAgent.rulesDir(1)); sin(currentAgent.rulesDir(1)), cos(currentAgent.rulesDir(1))];
                        CohMatrix      = [cos(currentAgent.rulesDir(2)), -sin(currentAgent.rulesDir(2)); sin(currentAgent.rulesDir(2)), cos(currentAgent.rulesDir(2))];
                        AliMatrix      = [cos(currentAgent.rulesDir(3)), -sin(currentAgent.rulesDir(3)); sin(currentAgent.rulesDir(3)), cos(currentAgent.rulesDir(3))];
                        MigMatrix      = [cos(currentAgent.rulesDir(4)), -sin(currentAgent.rulesDir(4)); sin(currentAgent.rulesDir(4)), cos(currentAgent.rulesDir(4))];
                        %WagMatrix      = [cos(currentAgent.rulesDir(5)), -sin(currentAgent.rulesDir(5)); sin(currentAgent.rulesDir(5)), cos(currentAgent.rulesDir(5))];

                        arrow = SL.Arrow;
                        arrow = arrow .* SL.renderScale .* 0.6;
                        scalingFactor = max(max(currentAgent.rulesMag),1);
                        Sarrow = (SepMatrix * arrow .* currentAgent.rulesMag(1) ./ scalingFactor)' + currentAgent.position(1:2);
                        Carrow = (CohMatrix * arrow .* currentAgent.rulesMag(2) ./ scalingFactor)' + currentAgent.position(1:2);
                        Aarrow = (AliMatrix * arrow .* currentAgent.rulesMag(3) ./ scalingFactor)' + currentAgent.position(1:2);
                        Marrow = (MigMatrix * arrow .* currentAgent.rulesMag(4) ./ scalingFactor)' + currentAgent.position(1:2);
                        %Warrow = (WagMatrix * arrow .* currentAgent.rulesMag(5) ./ scalingFactor)' + currentAgent.position(1:2);

                        %% Create Arrows
                        if(class(currentAgent.patchObj) == "double")
                            % obj.patchArr = patch('FaceColor',color);
                            obj.patchSep = patch('FaceColor',[1 1 0]); % Yellow
                            obj.patchCoh = patch('FaceColor',[1 0 1]); % Magenta
                            obj.patchAli = patch('FaceColor',[0 1 1]); % Cyan
                            obj.patchMig = patch('FaceColor',[1 1 1]); % White
                            %obj.patchWag = patch('FaceColor',[.5 .5 .5]); % Gray
                        end

                        %% Position Arrows
                        obj.patchSep.XData = Sarrow(:,1);
                        obj.patchCoh.XData = Carrow(:,1);
                        obj.patchAli.XData = Aarrow(:,1);
                        obj.patchMig.XData = Marrow(:,1);
                        %obj.patchWag.XData = Warrow(:,1);
        
                        obj.patchSep.YData = Sarrow(:,2);
                        obj.patchCoh.YData = Carrow(:,2);
                        obj.patchAli.YData = Aarrow(:,2);
                        obj.patchMig.YData = Marrow(:,2);
                        %obj.patchWag.YData = Warrow(:,2);

                    end
                    
                    %% Text Box
                    if(SL.showText)
                        textStr = sprintf('Speed: %2.0fm/s\nBank: %+3.0fdeg\nVSpeed: %1.1fm/s\nS:%2.2g\nC:%2.2g\nA:%2.2g\nM:%2.2g\nW:%2.2g',...
                            currentAgent.velocity(1), currentAgent.bankAngle*180/pi,currentAgent.savedVelocity(3), currentAgent.rulesMag(1),...
                            currentAgent.rulesMag(2),currentAgent.rulesMag(3),currentAgent.rulesMag(4),currentAgent.rulesMag(5));
                        if(class(obj.textAnnt) == "double")
                            obj.textAnnt = annotation('textbox');
                            obj.textAnnt.FontName = 'FixedWidth';
                            obj.textAnnt.BackgroundColor = [1 1 0];
                            obj.textAnnt.FaceAlpha = 0.75;
                            obj.textAnnt.Position = [0.55 0.75 0.25 0.15];
                            obj.textAnnt.FitBoxToText = 'on';
                            obj.textAnnt.FontSize = 10;
                        end
                        obj.textAnnt.String = textStr;

                    end
                end
                %% All Agents
                if ~SL.followAgent || (abs(obj.agents(i).position(1) - obj.agents(obj.thisAgent).position(1)) < SL.followRadius ...
                                    && abs(obj.agents(i).position(2) - obj.agents(obj.thisAgent).position(2)) < SL.followRadius)
                    obj.agents(i).render();
                elseif(class(obj.agents(i).patchObj) ~= "double")
                    obj.agents(i).patchObj.Visible = 'off';
                end
            end
        end
        
        % Get Data (probably outdated but dont want to delete
        function [maxHeight, minHeight, avgHeight, avgSpeed] = getData(obj)
            SL = obj.simLaw;
            
            maxHeightIndex = 0;
            maxHeight = SL.agentFloor;
            minHeightIndex = 0;
            minHeight = SL.agentCeiling;
            avgHeight = 0;
            avgSpeed = 0;
            
            for i=1:SL.numAgents
                currentHeight = obj.agents(i).position(3);
                if(currentHeight > maxHeight)
                    maxHeightIndex = i;
                    maxHeight = currentHeight;
                end
                if(currentHeight < minHeight)
                    minHeightIndex = i;
                    minHeight = currentHeight;
                end
                avgHeight = avgHeight + currentHeight;
                avgSpeed = avgSpeed + obj.agents(i).savedVelocity(1);
            end
            avgHeight = avgHeight / SL.numAgents;
            avgSpeed = avgSpeed / SL.numAgents;
        end
        
        % Updated Update Data Function
        function obj = updateData(obj,step)
            SL = obj.simLaw;
            
            obj.Elapsed(1) =     floor(step*SL.dt/3600);
            obj.Elapsed(2) = mod(floor(step*SL.dt/60  ),60);
            obj.Elapsed(3) = mod(floor(step*SL.dt     ),60);

            obj.maxHeight = SL.agentFloor;
            obj.minHeight = SL.agentCeiling;
            obj.avgHeight = 0;
            obj.avgSpeed = 0;
            
%             if Living ~= nnz([swarm.agents.isAlive])
%                 % if living is suddenly 39/40, update number 1 to whatever time it
%                 % is now.
%                 % if multiple agents die, update that number of elements.
%                 % Living should always be >= nnz of isAlive; isAlive updates first.
%                 ToD((SL.numAgents - Living + 1) : (SL.numAgents - nnz([swarm.agents.isAlive]))) = minutes;
%             end
            obj.Living  = nnz([obj.agents.isAlive]);
            obj.flightTime    = obj.flightTime + obj.Living * SL.dt;

            for i=1:SL.numAgents
                currentHeight = obj.agents(i).position(3);
                if(currentHeight > obj.maxHeight)
                    obj.maxHeight = currentHeight;
                end
                if(currentHeight < obj.minHeight)
                    obj.minHeight = currentHeight;
                end
                obj.avgHeight = obj.avgHeight + currentHeight;
                obj.avgSpeed = obj.avgSpeed + obj.agents(i).savedVelocity(1);
            end
            % THIS
            % obj.avgHeight = obj.avgHeight / SL.numAgents;
            % obj.avgSpeed = obj.avgSpeed / SL.numAgents;
            % OR THIS
            obj.avgHeight = obj.avgHeight / nnz([obj.agents.isAlive]);
            obj.avgSpeed = obj.avgSpeed / nnz([obj.agents.isAlive]);
        end
        
        % Render EVERYTHING
        function renderAll(obj)
%             obj.thermalMap = thermalMap;
            SL = obj.simLaw;
            hold on
    
            obj.thermalMap.renderThermals();
            obj.renderAgents();
            if SL.followAgent
                 xlim([obj.agents(swarm.thisAgent).position(1) - SL.followRadius, obj.agents(swarm.thisAgent).position(1) + SL.followRadius]);
                 ylim([obj.agents(swarm.thisAgent).position(2) - SL.followRadius, obj.agents(swarm.thisAgent).position(2) + SL.followRadius]);
            else
                xlim(SL.mapSize);
                ylim(SL.mapSize);
            end
            ax = gca;
            ax.PositionConstraint = 'outerposition';

            currFrame = getframe(obj.simFig);
            writeVideo(obj.video,currFrame);
            pause(0.0001);
    
    %         stringTitle = sprintf("Agents Alive: %g\nMax Height: %.1f\nMin Height: %.1f\nAverage Height: %.1f",Living,maxHeight,minHeight,averageHeight);
    %         stringTitle = sprintf("Minutes: %g\nAgents Alive: %g\nAverage Height: %.1f",minutes,Living,averageHeight);
            stringTitle = sprintf("Number %g, T+%01g:%02g:%02g, Score = %5.0fs\nLiving: %g  Avg: %.0f Min: %.0f Max: %.0f", ...
                obj.number, obj.Elapsed(1), obj.Elapsed(2), obj.Elapsed(3),obj.flightTime, obj.Living, obj.avgHeight, obj.minHeight, obj.maxHeight);
            title(stringTitle);
            hold off

        end

        % Initialize Video
        function obj = initVideo(obj,videoName)
            
            SL = obj.simLaw;
            obj.video = VideoWriter(videoName);
            obj.video.FrameRate = 1/SL.dt * SL.fpsMult / SL.frameSkip;
            open(obj.video);
            
            obj.simFig = figure('Visible','on');
        
            % Initialize map background
            clf
            xlim(SL.mapSize);
            ylim(SL.mapSize);
            daspect([1 1 1]);
            colorbar;
            cbLimits = [-1,SL.thermalStrengthMax];
            colors = [6 42 127; 41 76 247; 102 59 231; 162 41 216; 222 24 200; 255 192 203] / 255;
            x = [0:obj.thermalMap.thermalPixels/(length(colors)-1):obj.thermalMap.thermalPixels];
            map = interp1(x/obj.thermalMap.thermalPixels,colors,linspace(0,1,obj.thermalMap.thermalPixels)); % Creates a color gradient for the map
            colormap();%map);
            set(gca,'clim',cbLimits);

        end
        
        % Close Video
        function closeVideo(obj)
            close(obj.video);
        end
    end
end