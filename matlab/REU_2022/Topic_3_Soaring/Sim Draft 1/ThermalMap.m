% Thermal map class: stores list of Thermals
classdef ThermalMap < handle
    properties
        thermalSizeLims = [5 20];
        thermalVelLims = [20 50];
        thermals = Thermal.empty(0,SimLaw.numThermals);
    end

    methods (Static)
        % Default construcor
        function thermalMap = ThermalMap() 
            thermalMap.thermals(1,SimLaw.numThermals) = Thermal();

            for i = 1:SimLaw.numThermals
                % Randomize the thermal's properties
                thermalMap.thermals(i).position(1) = Utility.randIR(SimLaw.mapSize(1),SimLaw.mapSize(2));
                thermalMap.thermals(i).position(2) = Utility.randIR(SimLaw.mapSize(1),SimLaw.mapSize(2));
                        
                % Randomly decide if the velocity is negative or positive
                randFactor = randi([0 1],1,2);
                randFactor(randFactor == 0) = -1;
                thermalMap.thermals(i).velocity(1) = Utility.randIR(thermalMap.thermalVelLims(1),thermalMap.thermalVelLims(2))*randFactor(1);
                thermalMap.thermals(i).velocity(2) = Utility.randIR(thermalMap.thermalVelLims(1),thermalMap.thermalVelLims(2))*randFactor(2);
                        
                thermalMap.thermals(i).radius = Utility.randIR(thermalMap.thermalSizeLims(1),thermalMap.thermalSizeLims(2));
                thermalMap.thermals(i).strength = 1;
            end
        end

        % Calculate updraft strength at a given point
        function strength = getStrength(thermal, position)
            % determine distance to all thermals
            distTherm = zeros(1,numThermals);
            for i = 1:numThermals
                distTherm(i) = norm(position - thermal(1).position);
            end

            % check which thermal we are in. Returns one number or empty.
            inTh = find(distTherm <= thermal.radius);
            
            % currently assumes strength is the same at all altitudes
            strength = thermal.strength*exp(-(3*distTherm(inTh)/thermal.radius)^2)*...
                                          (1-(3*distTherm(inTh)/thermal.radius)^2);s
        end

        % Ensure thermals don't overlap
        function thermals = adjustThermalPositions(thermals)
            for i = 1:SimLaw.numThermals
                for j = (i + 1):SimLaw.numThermals
                    % Calculate if the thermals overlap
                    distance = norm(thermals(i).position - thermals(j).position);
                    if distance <= (thermals(i).radius + thermals(j).radius)
                        % Needs a better way to change direction
                        thermals(i).velocity(1) = -thermals(i).velocity(1);
                        thermals(j).velocity(1) = -thermals(j).velocity(1);
                    end
                end
            end
        end

    end
end