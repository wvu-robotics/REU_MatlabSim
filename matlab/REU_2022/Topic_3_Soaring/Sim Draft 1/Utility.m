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
        %% Calculates Weighted Centroid
        function [Centroid, distances, diffHeight] = findCentroid(currentAgent, localAgents, SL)
            %% Init
            numLocalAgents = size(localAgents,2);
            Centroid       = [0,0,0];
            distances      = zeros(1,numLocalAgents);
            diffHeight     = distances;
            numLocalAgents = length(distances);
            
            %% Loop
            for i = 1:numLocalAgents
                if ~localAgents(i).isAlive
                    continue;
                end
                distances(i) = norm(currentAgent.position - localAgents(i).savedPosition);
                diffHeight(i) = -currentAgent.position(3) + localAgents(i).savedPosition(3); % negative if above others.
                normHeight = diffHeight(i)/SL.neighborRadius;
                if normHeight < SL.heightIgnore
                    weight = 0;
                else
                    weight = SL.heightPriority * (normHeight - SL.heightIgnore);
                    % if normHeight = 1 and heightIgnore = -1, weight is 2.
                end
                Centroid = Centroid + weight*localAgents(i).savedPosition;
            end
            Centroid = Centroid / numLocalAgents;  
        end

        %% Mid of Min and Max
        function mid = midMinMax(num, min, max)
            mid = min(max(num,min),max);
        end
        %% Render From Data (WIP)
        function video = renderData(fileName)
            allData = load(fileName,"xData","yData","zData","fVelData","VelData","zVelData","bankAngleData","headingData");
            numSteps = allData.SL.totalTime / allData.SL.dt;
        end
    end
end