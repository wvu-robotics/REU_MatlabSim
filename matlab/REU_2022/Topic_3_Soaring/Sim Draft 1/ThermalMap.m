% Thermal map class: stores list of Thermals
classdef ThermalMap < handle
    properties
        thermalSizeLims = [5 20];
        thermalVelLims = [20 50];
        thermals
        simLaw
    end

    methods 
        % Default construcor
        function thermalMap = ThermalMap(simLaw) 
            thermalMap.simLaw = simLaw;
            SL = thermalMap.simLaw;
            
            thermalMap.thermals = Thermal.empty(0,SL.numThermals);

            for i = 1:SL.numThermals
                thermalMap.thermals(i) = Thermal(SL);
            end
        end

        % Calculate updraft strength at a given point
        function strength = getStrength(thermalMap, position, i)
            SL = thermalMap.simLaw;
            % determine distance to all thermals
            %strength = 0;
           % for i = 1:SL.numThermals
                radius = thermalMap.thermals(i).radius;
                %distTherm = norm((position / 5) - 100 - thermalMap.thermals(i).position);
                distTherm = norm(thermalMap.thermals(i).position - position);
         
                % If the point is inside the thermal, calculate its strength
               % if distTherm(i) <= thermalMap.thermals(i).radius
                    % currently assumes strength is the same at all altitudes
                    x = exp(-(3*distTherm/radius)^2);
                    y = (1-(3*distTherm/radius)^2);
                    strength = thermalMap.thermals(i).curStrength.*x.*y;
               % end
           % end
        end

        % Ensure thermals don't overlap
        function [] = adjustThermalPositions(thermalMap)
            SL = thermalMap.simLaw;
            for i = 1:SL.numThermals
                for j = (i + 1):SL.numThermals
                    % Calculate if the thermals overlap
                    distance = norm(thermalMap.thermals(i).position - thermalMap.thermals(j).position);
                    if distance <= (thermalMap.thermals(i).radius * (thermalMap.thermals(i).curStrength/thermalMap.thermals(i).maxStrength) + thermalMap.thermals(j).radius * (thermalMap.thermals(j).curStrength/thermalMap.thermals(j).maxStrength))
                        % Determine which thermal is weaker
                        if (thermalMap.thermals(i).curStrength < thermalMap.thermals(j).curStrength) 
                            weakerThermal = i;
                        else
                            weakerThermal = j;
                        end

                        thermalMap.thermals(weakerThermal).curStrength = 0;
                        thermalMap.thermals(weakerThermal) = Thermal(SL);

                        thermalMap.fadeThermals(weakerThermal);
                        % Needs a better way to change direction
                        thermalMap.thermals(i).velocity(1) = -thermalMap.thermals(i).velocity(1);
                        thermalMap.thermals(j).velocity(1) = -thermalMap.thermals(j).velocity(1);
                    end
                end
            end
        end
        
        % Fade the thermals in or out depending on the time
        function [] = fadeThermals(thermalMap, i)
            SL = thermalMap.simLaw;
            % If the current strength is at the thermal's max, determine the step count
            if thermalMap.thermals(i).curStrength == thermalMap.thermals(i).maxStrength
                % If the step count is at the plateau time, reset it to 0, reverse the strength direction, and increment the
                % current strength in the new direction
                if thermalMap.thermals(i).stepCount == SL.thermalMaxPlateauTime
                    thermalMap.thermals(i).stepCount = 0;
                    thermalMap.thermals(i).strengthDirection = -thermalMap.thermals(i).strengthDirection;
                    thermalMap.thermals(i).curStrength = thermalMap.thermals(i).curStrength + thermalMap.thermals(i).strengthDirection * SL.thermalFadeRate;
                else % Otherwise, increment the step count
                    thermalMap.thermals(i).stepCount = thermalMap.thermals(i).stepCount + 1;
                end
            % Otherwise check if the current strength is at 0
            elseif thermalMap.thermals(i).curStrength <= 0
                if thermalMap.thermals(i).stepCount == SL.thermalMinPlateauTime
                    thermalMap.thermals(i).stepCount = 0;
                    thermalMap.thermals(i).strengthDirection = -thermalMap.thermals(i).strengthDirection;
                    thermalMap.thermals(i).curStrength = thermalMap.thermals(i).curStrength + thermalMap.thermals(i).strengthDirection * SL.thermalFadeRate;
                else % Otherwise, increment the step count
                    thermalMap.thermals(i).stepCount = thermalMap.thermals(i).stepCount + 1;
                end
            else % Otherwise, increment the current strength in the same direction
                thermalMap.thermals(i).curStrength = thermalMap.thermals(i).curStrength + thermalMap.thermals(i).strengthDirection * SL.thermalFadeRate;
            end
        end

        function [] = checkBounds(thermalMap, thermalIndex) 
            SL = thermalMap.simLaw;
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
    end
end