%% Time Until Collision

%Description: Given a centralAgent and its neighbors, returns the times
%until they collide.

%Parameters: agent: A 1x1 Agent handle

%Returns:
%   times: An Nx1 double where the time until agent and the ith
%       measuredAgent collide is times(i). If they won't collide, then
%       times(i) = NaN. If they are colliding currently, then
%       times(i) = 0.
function times = timeTilCollision(agent)
    %Allocates all times until collisions
    times = zeros(length(agent.measuredAgents),1);
    
    %For each measuredAgent
    for i = 1:length(agent.measuredAgents)
    
        %Initializes variables for quadratic formula application
        p = agent.pose - agent.measuredAgents(i).pose;
        v = agent.velocity - agent.measuredAgents(i).velocity;
        r = agent.getRadius() + agent.measuredAgents(i).getRadius();
        
        %If their relative velocity is zero, the quadratic formula below
        %doesn't hold, so this case must be considered separately.
        if norm(v) == 0
            
            %If they are currently colliding
            if p < r
                times(i) = 0;
            else %If they aren't
                times(i) = NaN;
            end
            
        %If the time bounds on the collision period are complex or equal
        elseif dot(v,p)^2 - norm(v)^2 * (norm(p)^2 - r^2) <= 0
            %They won't ever collide
            times(i) = NaN;

        else
            %Calculates the times that bound their collision period.
            %They will collide at any time t in (time1,time2).
            time1 = ( -dot(v,p) - sqrt(dot(v,p)^2 - norm(v)^2 * (norm(p)^2 - r^2)) ) / (norm(v)^2);
            time2 = ( -dot(v,p) + sqrt(dot(v,p)^2 - norm(v)^2 * (norm(p)^2 - r^2)) ) / (norm(v)^2);

            %If both times are non-positive
            if time2 <= 0
                %They are moving away from a collision
                times(i) = NaN;

            %If time1 < 0 < time2
            elseif time1 < 0
                %They are in the middle of a collision
                times(i) = 0;

            %If 0 <= time1 < time2
            else
                %They will collide in the future
                times(i) = time1;
            end
        end
    end
end

