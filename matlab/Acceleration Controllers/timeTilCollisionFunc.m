%% Time Until Collision

%Description: Given a centralAgent and its neighbors, returns the times
%until they collide.

%Parameters:
%   centralAgentPosition & centralAgentVelocity: These are 1x2 doubles that
%       describe the state of centralAgent.
%   neighborsPositions & neighborsVelocities: These are Nx2 doubles where the
%       ith neighbor that centralAgent can see has position
%       nieghborsPositions(i,:) and has velocity neighborsVelocities(i,:).
%   agentRadius: A positive double for the radii of agents

%Returns:
%   times: An Nx1 double where the time until centralAgent and the ith
%       neighbor collide is times(i). If they won't collide, then
%       times(i) = NaN. If they are colliding currently, then
%       times(i) = .0001.
function times = timeTilCollisionFunc(centralAgentPosition, centralAgentVelocity, neighborsPositions, neighborsVelocities, agentRadius)
    %Allocates all times until collisions
    times = zeros(size(neighborsPositions,1),1);
    
    %Initializes variables for quadratic formula application
    p = centralAgentPosition - neighborsPositions;
    v = centralAgentVelocity - neighborsVelocities;
    r = 2 * agentRadius;
    
    %For each neighbor
    for i = 1:size(neighborsPositions,1)
    
        %If their relative velocity is zero, the quadratic formula below
        %doesn't hold, so this case must be considered separately.
        if norm(v(i,:)) == 0
            
            %If they are currently colliding
            if p(i,:) < r
                times(i) = .0001;
            else %If they aren't
                times(i) = NaN;
            end
            
        %If the time bounds on the collision period are complex or equal
        elseif dot(v(i,:),p(i,:))^2 - norm(v(i,:))^2 * (norm(p(i,:))^2 - r^2) <= 0
            %They won't ever collide
            times(i) = NaN;

        else
            %Calculates the times that bound their collision period.
            %They will collide at any time t in (time1,time2).
            time1 = ( -dot(v(i,:),p(i,:)) - sqrt(dot(v(i,:),p(i,:))^2 - norm(v(i,:))^2 * (norm(p(i,:))^2 - r^2)) ) / (norm(v(i,:))^2);
            time2 = ( -dot(v(i,:),p(i,:)) + sqrt(dot(v(i,:),p(i,:))^2 - norm(v(i,:))^2 * (norm(p(i,:))^2 - r^2)) ) / (norm(v(i,:))^2);

            %If both times are non-positive
            if time2 <= 0
                %They are moving away from a collision
                times(i) = NaN;

            %If time1 < 0 < time2
            elseif time1 < .0001
                %They are in the middle of a collision
                times(i) = .0001;

            %If 0 <= time1 < time2
            else
                %They will collide in the future
                times(i) = time1;
            end
        end
    end
end

