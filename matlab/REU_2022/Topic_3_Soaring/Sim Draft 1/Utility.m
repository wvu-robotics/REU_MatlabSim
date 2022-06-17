% Utility class: stores helper functions
classdef Utility
    methods (Static)
        %% Calculates Random number In Range (randIR) between low and high
        function value = randIR(low,high) 
            value = rand() * (high-low) + low;
        end
        
        %% Calculates distance if within threshold
        function [Verdict] = isNear(A1, A2, Threshold)
            % A NaN Threshold will tell isNear to ignore the threshold.
            if ~isnan(Threshold)
                Verdict = NaN;
                if abs(A1.position(1) - A2.position(1)) > Threshold 
                    return
                elseif abs(A1.position(2) - A2.position(2)) > Threshold
                    return
                elseif abs(A1.position(3) - A2.position(3)) > Threshold
                    return
                elseif A1.position(1) == A2.position(1) && A1.position(2) == A2.position(2)
                    return
                end
            end
            dist = norm([(A1.position(1)-A2.position(1)), ...
                         (A1.position(2)-A2.position(2)), ...
                         (A1.position(3)-A2.position(3))]);
            if isnan(Threshold) || dist <= Threshold
                Verdict = dist;
            end
        end
        %% Calculates Weighted Centroid (WIP, don't use)

        function Centroid = findCentroid(currentAgent,localAgents)
            numLocalAgents = size(localAgents,2);
            Centroid = [0,0,0];
            distances = zeros(1,numLocalAgents);
            diffHeight = distances;
            numLocalAgents = length(distances);
            for i = 1:numLocalAgents
                if localAgents(i).savedPosition(3) <= 0
                    continue;
                end
                distances(i) = norm(currentAgent.position - localAgents(i).savedPosition);
                diffHeight(i) = -currentAgent.position(3) + localAgents(i).savedPosition(3); % negative if above others.
                normHeight = diffHeight(i)/SL.neighborRadius;
                if normHeight < SL.heightIgnore
                    weight = 0;
                else
                    weight = SL.heightPriority * (normHeight - SL.heightIgnore);
                    % if normHeight = 0 and heightOffset = -0.4, weight is 0.4
                    % if normHeight = 1 and heightOffset = -1, weight is 2.
                end
                centroid = centroid + weight*localAgents(i).savedPosition;
            end
            Centroid = Centroid / numLocalAgents;  
        end

        %% Mid of Min and Max
        function mid = midMinMax(num, min, max)
            if num > max
                num = max;
            elseif num < min
                num = min;
            end
            mid = num;
        end
    end
end