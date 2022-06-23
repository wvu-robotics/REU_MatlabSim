% Thermal map class: stores list of Thermals
classdef ThermalMap < handle
    properties
        thermalSizeLims = [5 20];
        thermalVelLims = [20 50];
        thermals
        thermalPixels = 200;
        simLaw
        
        thermalMapImg = NaN
    end

    methods 
        % Constructor
        function thermalMap = ThermalMap(simLaw)
            thermalMap.simLaw = simLaw;
            thermalMap.thermalPixels = simLaw.thermalPixels;
            thermalMap.thermals = Thermal.empty(0,simLaw.numThermals);
            
            for i = 1:simLaw.numThermals
                thermalMap.thermals(i) = Thermal(simLaw);
            end
        end

        %% Return true if the two input thermals overlap, return false otherwise
        function overlapBool = isOverlap(thermalMap, index1, index2)
            distance = norm(thermalMap.thermals(index1).position - thermalMap.thermals(index2).position);
            overlapBool = (distance <= (thermalMap.thermals(index1).radius + thermalMap.thermals(index2).radius));
        end

        %% Check for overlap between one thermal and all the thermals following it
        function index = checkOverlap(thermalMap, index, vel)
            SL = thermalMap.simLaw;
            for i = (index + 1):SL.numThermals
                if isOverlap(thermalMap, index, i)
                    reinitWeakThermal(thermalMap, index, i, vel);
                    index = 1;
                    break;
                end
            end
        end

        %% Determine weaker thermal
        function [] = reinitWeakThermal(thermalMap, index1, index2, vel)
            SL = thermalMap.simLaw;
            % Determine which thermal is weaker
            if (thermalMap.thermals(index1).curStrength <= thermalMap.thermals(index2).curStrength) 
                weakerThermal = index1;
            else
                weakerThermal = index2;
            end
            
            % Make the weaker thermal disappear and reincarnate it
            if vel ~= [0 0]
                thermalMap.thermals(weakerThermal) = Thermal(SL);
            else 
                thermalMap.thermals(weakerThermal) = Thermal(SL, 0);
            end
            thermalMap.thermals(weakerThermal).curStrength = 0;
            thermalMap.thermals(weakerThermal).stepCount = SL.thermalMinPlateauTime;
        end

        %% Determine which thermal a point is in
        function thermalIndex = findThermalIndex(thermalMap, position)
            SL = thermalMap.simLaw;
            thermalIndex = 0;
            for i = 1:SL.numThermals
                % If the distance from the thermal center to the point
                % is less than the radius, the point is in the thermal
                distance = norm(thermalMap.thermals(i).position - position(1:2));
                if distance < thermalMap.thermals(i).radius
                    thermalIndex = i;
                    break;
                end
            end
        end

        %% Calculate updraft strength at a given point
        function strength = getStrength(thermalMap, position, i)
            % If the thermal index isn't specified, call findThermalIndex
            if nargin == 2
                i = findThermalIndex(thermalMap, position);
            end
            if i == 0
                strength = 0;
                return;
            end

            radius = thermalMap.thermals(i).radius;
            % determine distance to the given thermals
            distTherm = norm(thermalMap.thermals(i).position - position(1:2));
            
            % Calculate the updraft strength
            x = exp(-(3*distTherm/radius)^2);
            y = (1-(3*distTherm/radius)^2);
            strength = thermalMap.thermals(i).curStrength.*x.*y;
            
        end

        %% Fade the thermals in or out depending on the time
        function [] = fadeThermals(thermalMap)
            SL = thermalMap.simLaw;
            for i = 1:SL.numThermals
                % If the current strength is at the thermal's max, determine the step count
                if thermalMap.thermals(i).curStrength >= (thermalMap.thermals(i).maxStrength - 1E-5)
                    % If the step count is at the plateau time, reset it to 0 and increment the
                    % current strength in the new direction
                    if thermalMap.thermals(i).stepCount == SL.thermalMaxPlateauTime
                        thermalMap.thermals(i).strengthDirection = -1;
                        thermalMap.thermals(i).stepCount = 0;       
                        thermalMap.thermals(i).curStrength = thermalMap.thermals(i).curStrength + thermalMap.thermals(i).strengthDirection * SL.thermalFadeRate;
                    else % Otherwise, increment the step count
                        thermalMap.thermals(i).stepCount = thermalMap.thermals(i).stepCount + 1;
                    end
                % Otherwise check if the current strength is at 0
                elseif round(thermalMap.thermals(i).curStrength,1) <= 1E-5                                    
                    if thermalMap.thermals(i).stepCount == SL.thermalMinPlateauTime
                        thermalMap.thermals(i).strengthDirection = 1;
                        thermalMap.thermals(i).stepCount = 0;
                        thermalMap.thermals(i).curStrength = thermalMap.thermals(i).curStrength + thermalMap.thermals(i).strengthDirection * SL.thermalFadeRate;
                    else % Otherwise, increment the step count
                        thermalMap.thermals(i).stepCount = thermalMap.thermals(i).stepCount + 1;
                    end
                else % Otherwise, increment the current strength in the same direction
                    thermalMap.thermals(i).curStrength = thermalMap.thermals(i).curStrength + thermalMap.thermals(i).strengthDirection * SL.thermalFadeRate;
                end
            end
        end

        %% Move thermals that hit their bounds
        function [] = checkBounds(thermalMap, thermalIndex) 
            % Check the left and right bounds of the map: if the thermal hits
            % one, move it in the opposite direction
            thermalList = thermalMap.thermals;
            if(thermalList(thermalIndex).position(1) >= thermalList(thermalIndex).bounds(2))
                thermalList(thermalIndex).position(1) = 2*thermalList(thermalIndex).bounds(2) - thermalList(thermalIndex).position(1);
                thermalList(thermalIndex).velocity(1) = -thermalList(thermalIndex).velocity(1);
            elseif(thermalList(thermalIndex).position(1) <= thermalList(thermalIndex).bounds(1))
                thermalList(thermalIndex).position(1) = 2*thermalList(thermalIndex).bounds(1) - thermalList(thermalIndex).position(1);
                thermalList(thermalIndex).velocity(1) = -thermalList(thermalIndex).velocity(1);
            end
            
            % Check the upper and lower bounds of the map
            if(thermalList(thermalIndex).position(2) >= thermalList(thermalIndex).bounds(2))
                thermalList(thermalIndex).position(2) = 2*thermalList(thermalIndex).bounds(2) - thermalList(thermalIndex).position(2);
                thermalList(thermalIndex).velocity(2) = -thermalList(thermalIndex).velocity(2);
            elseif(thermalList(thermalIndex).position(2) <= thermalList(thermalIndex).bounds(1))
                thermalList(thermalIndex).position(2) = 2*thermalList(thermalIndex).bounds(1) - thermalList(thermalIndex).position(2);
                thermalList(thermalIndex).velocity(2) = -thermalList(thermalIndex).velocity(2);
            end
        end

        %% Render thermals
        function renderThermals(thermalMap)
            SL = thermalMap.simLaw;
            % Initialize background of map
            mapX = linspace(SL.mapSize(1),SL.mapSize(2),thermalMap.thermalPixels);
            mapY = linspace(SL.mapSize(1),SL.mapSize(2),thermalMap.thermalPixels);
            mapDiff = (SL.mapSize(2)-SL.mapSize(1))/(thermalMap.thermalPixels-1);
            
            finalThermalMap = zeros(thermalMap.thermalPixels);
            % Iterate through thermals
            for thermalIndex = 1:SL.numThermals
                thermalPos = thermalMap.thermals(thermalIndex).position;
                thermalRad = thermalMap.thermals(thermalIndex).radius;
                
                % Square bounds in pixels around the thermal center: min is bottom
                % left and max is top right
                thermalSquareMin = [thermalPos(1)-thermalRad,thermalPos(2)-thermalRad];
                thermalSquareMax = [thermalPos(1)+thermalRad,thermalPos(2)+thermalRad];
                
                % Calculate the position of the thermal square on the map
                mapPosMin = [round((thermalSquareMin(1)-SL.mapSize(1))/mapDiff),round((thermalSquareMin(2)-SL.mapSize(1))/mapDiff)];
                mapPosMax = [round((thermalSquareMax(1)-SL.mapSize(1))/mapDiff),round((thermalSquareMax(2)-SL.mapSize(1))/mapDiff)];
                
                % Create temporary empty matrix to hold strengths around this thermal
                tempThermalMap = zeros(thermalMap.thermalPixels);
                % Iterate through the indices in the thermal square
                for row = mapPosMin(2):mapPosMax(2)
                    for column = mapPosMin(1):mapPosMax(1)
                        % For each pixel in the square, find its strength relative
                        % to the thermal
                        tempThermalMap(row, column) = thermalMap.getStrength([mapY(column),mapX(row)], thermalIndex);
                    end
                end
                finalThermalMap = finalThermalMap + tempThermalMap;
            end
            
            if(class(thermalMap.thermalMapImg) == "double")
                thermalMap.thermalMapImg = image('CData',finalThermalMap,'XData',SL.mapSize,'YData',SL.mapSize,'CDataMapping','scaled');
                thermalMap.thermalMapImg.AlphaData = 1;
            else
                thermalMap.thermalMapImg.CData = finalThermalMap;
            end
        end

        %% Step functions
        % Step for location varying simulation
        function [] = step(thermalMap, dt)
            SL = thermalMap.simLaw;
            for thermalIndex = 1:SL.numThermals
                thermalMap.adjustThermalPositions(thermalMap.thermals(thermalIndex).velocity);
                thermalMap.fadeThermals();
                thermalList = thermalMap.thermals;
        
                % Update the position of the thermal
                thermalList(thermalIndex).position(1) = thermalList(thermalIndex).position(1) + thermalList(thermalIndex).velocity(1)*dt;
                thermalList(thermalIndex).position(2) = thermalList(thermalIndex).position(2) + thermalList(thermalIndex).velocity(2)*dt;
                
                thermalMap.checkBounds(thermalIndex);
            end
        end

        % Step for non-location varying map
        function [] = staticStep(thermalMap)
            SL = thermalMap.simLaw;
            for thermalIndex = 1:SL.numThermals
                thermalMap.adjustThermalPositions(0);
                thermalMap.fadeThermals();
            end
        end

        %% Adjust thermal positions if they overlap
        function [] = adjustThermalPositions(thermalMap, vel)
            SL = thermalMap.simLaw;
            for i = 1:SL.numThermals
                i = checkOverlap(thermalMap, i, vel);
            end
        end
    end
end