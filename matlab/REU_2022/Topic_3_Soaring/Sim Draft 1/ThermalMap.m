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
                thermalMap.thermals(i).position(1) = Utility.randIR(SimLaw.mapSize(1),SimLaw.mapSize(2));
                thermalMap.thermals(i).position(2) = Utility.randIR(SimLaw.mapSize(1),SimLaw.mapSize(2));
                        
                thermalMap.thermals(i).velocity(1) = Utility.randIR(thermalMap.thermalVelLims(1),thermalMap.thermalVelLims(2))*randi([-1 1],1);
                thermalMap.thermals(i).velocity(2) = Utility.randIR(thermalMap.thermalVelLims(1),thermalMap.thermalVelLims(2))*randi([-1 1],1);
                        
                thermalMap.thermals(i).radius = Utility.randIR(thermalMap.thermalSizeLims(1),thermalMap.thermalSizeLims(2));
                thermalMap.thermals(i).strength = 1;
            end
        end

        % Calculate updraft strength at a given point
        %function strength = getStrength(position)
            
        %end

        % Ensure thermals don't overlap
        function newMap = adjustThermalPositions()
            for i = 1:SimLaw.numThermals
                
            end
        end
    end
end