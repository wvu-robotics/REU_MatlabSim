% Thermal map class: stores list of Thermals
classdef ThermalMap < handle
    properties
        thermalSizeLims = [5 20];
        thermalVelLims = [20 50];
        thermals = Thermal.empty(0,SimLaw.numThermals);
    end

    methods 
        % Default construcor
        function thermalMap = ThermalMap() 
            thermalMap.thermals(1,SimLaw.numThermals) = Thermal();

            for i = 1:SimLaw.numThermals
                % Randomize the thermal's properties
                thermalMap.thermals(i).position(1) = round(Utility.randIR(SimLaw.mapSize(1) + 20,SimLaw.mapSize(2) - 20));
                thermalMap.thermals(i).position(2) = round(Utility.randIR(SimLaw.mapSize(1) + 20,SimLaw.mapSize(2) - 20));
                        
                % Randomly decide if the velocity is negative or positive
                randFactor = randi([0 1],1,2);
                randFactor(randFactor == 0) = -1;
                thermalMap.thermals(i).velocity(1) = Utility.randIR(SimLaw.thermalSpeedMin,SimLaw.thermalSpeedMax)*randFactor(1);
                thermalMap.thermals(i).velocity(2) = Utility.randIR(SimLaw.thermalSpeedMin,SimLaw.thermalSpeedMax)*randFactor(2);
                        
                thermalMap.thermals(i).radius = round(Utility.randIR(SimLaw.thermalRadiusMin,SimLaw.thermalRadiusMax));
                thermalMap.thermals(i).maxStrength = round(Utility.randIR(SimLaw.thermalStrengthMin + 1,SimLaw.thermalStrengthMax));
                thermalMap.thermals(i).curStrength = round(Utility.randIR(1, thermalMap.thermals(i).maxStrength));
            end
        end

        % Calculate updraft strength at a given point
        function strength = getStrength(thermalMap, position, i)
            % determine distance to all thermals
            %strength = 0;
           % for i = 1:SimLaw.numThermals
                radius = thermalMap.thermals(i).radius;
                %distTherm = norm((position / 5) - 100 - thermalMap.thermals(i).position);
                distTherm = norm(thermalMap.thermals(i).position - position);
         
                % If the point is inside the thermal, calculate its strength
               % if distTherm(i) <= thermalMap.thermals(i).radius
                    % currently assumes strength is the same at all altitudes
                    x = exp(-(distTherm/radius)^2);
                    y = (1-(distTherm/radius)^2);
                    strength = thermalMap.thermals(i).curStrength.*x.*y;
               % end
           % end
        end

        % Ensure thermals don't overlap
        function [] = adjustThermalPositions(thermalMap)
            for i = 1:SimLaw.numThermals
                for j = (i + 1):SimLaw.numThermals
                    % Calculate if the thermals overlap
                    distance = norm(thermalMap.thermals(i).position - thermalMap.thermals(j).position);
                    if distance <= (thermalMap.thermals(i).radius + thermalMap.thermals(j).radius + 10)
                        % Needs a better way to change direction
                        thermalMap.thermals(i).velocity(1) = -thermalMap.thermals(i).velocity(1);
                        thermalMap.thermals(j).velocity(1) = -thermalMap.thermals(j).velocity(1);
                    end
                end
            end
        end
        
        % Fade the thermals in or out depending on the time
        function [] = fadeThermals(thermalMap)
            for i = 1:SimLaw.numThermals
                % If the current strength is at the max or min, determine the step count
                if thermalMap.thermals(i).curStrength == thermalMap.thermals(i).maxStrength || thermalMap.thermals(i).curStrength == 0
                    % If the step count is at the plateau time, reset it to 0, reverse the strength direction, and increment the
                    % current strength in the new direction
                    if thermalMap.thermals(i).stepCount == SimLaw.thermalPlateauTime
                        thermalMap.thermals(i).stepCount = 0;
                        thermalMap.thermals(i).strengthDirection = -thermalMap.thermals(i).strengthDirection;
                        thermalMap.thermals(i).curStrength = thermalMap.thermals(i).curStrength + thermalMap.thermals(i).strengthDirection * SimLaw.thermalFadeRate;
                    else % Otherwise, increment the step count
                        thermalMap.thermals(i).stepCount = thermalMap.thermals(i).stepCount + 1;
                    end
                else % Otherwise, increment the current strength in the same direction
                    thermalMap.thermals(i).curStrength = thermalMap.thermals(i).curStrength + thermalMap.thermals(i).strengthDirection * SimLaw.thermalFadeRate;
                end
            end
        end

        function [] = checkBounds(thermalMap, thermalIndex) 
            % Check the left and right bounds of the map: if the thermal hits
            % one, move it in the opposite direction
            thermalList = thermalMap.thermals;
            if(thermalList(thermalIndex).position(1) >= SimLaw.thermalBounds(2))
                thermalList(thermalIndex).position(1) = 2*SimLaw.thermalBounds(2) - thermalList(thermalIndex).position(1);
                thermalList(thermalIndex).velocity(1) = -thermalList(thermalIndex).velocity(1);
            elseif(thermalList(thermalIndex).position(1) <= SimLaw.thermalBounds(1))
                thermalList(thermalIndex).position(1) = 2*SimLaw.thermalBounds(1) - thermalList(thermalIndex).position(1);
                thermalList(thermalIndex).velocity(1) = -thermalList(thermalIndex).velocity(1);
            end
            
            % Check the upper and lower bounds of the map
            if(thermalList(thermalIndex).position(2) >= SimLaw.thermalBounds(2))
                thermalList(thermalIndex).position(2) = 2*SimLaw.thermalBounds(2) - thermalList(thermalIndex).position(2);
                thermalList(thermalIndex).velocity(2) = -thermalList(thermalIndex).velocity(2);
            elseif(thermalList(thermalIndex).position(2) <= SimLaw.thermalBounds(1))
                thermalList(thermalIndex).position(2) = 2*SimLaw.thermalBounds(1) - thermalList(thermalIndex).position(2);
                thermalList(thermalIndex).velocity(2) = -thermalList(thermalIndex).velocity(2);
            end
        end
    end
end