% Utility class: stores helper functions
classdef Utility
    methods (Static)
        %Calculates Random number In Range (randIR) between low and high
        function value = randIR(low,high) 
            value = rand() * (high-low) + low;
        end
        
        % Calculates distance if within threshold.
        function [Verdict] = isNear(A1, A2, Threshold)
            Verdict = NaN;
            if abs(A1.position(1) - A2.position(1)) > Threshold 
                return
            elseif abs(A1.position(2) - A2.position(2)) > Threshold
                return
            end
            
            dist = norm([(A1.position(1)-A2.position(1)), (A1.position(2)-A2.position(2))]);
            if dist <= Threshold
                Verdict = dist;
            end
        end
    end
end