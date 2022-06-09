% Utility class: stores helper functions
classdef Utility
    methods (Static)
        %Calculates Random number In Range (randIR) between low and high
        function value = randIR(low,high) 
            value = rand() * (high-low) + low;
        end
    end
end