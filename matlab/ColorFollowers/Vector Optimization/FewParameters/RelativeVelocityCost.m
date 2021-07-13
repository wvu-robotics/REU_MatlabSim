function deltaCost = RelativeVelocityCost(agents)
deltaCost = 0;

for i = 1:length(agents)
    for j = 1:length(agents)
        relativeDist = norm(agents(i).pose - agents(j).pose);
        if relativeDist < agents(i).measuringRange && j ~= i
            relativeSpeed = norm(agents(i).velocity - agents(j).velocity);
            %There is a big benefit for reaching the goal and then you
            %don't care about how close others are
            if norm(agents(i).pose - agents(i).goalPose) < agents(i).maxSpeed
                deltaCost = deltaCost - 1000; %The value of 10 is just arbitrary
            else
            %The delta cost is larger magnitude as the neighboring agents
            %get closer
            %The delta cost is negative if the relativeSpeed is below 1/2
            %of the agents max speed
            deltaCost = deltaCost + (relativeSpeed - agents(i).maxSpeed * 0.5) * (agents(i).measuringRange - relativeDist);
            end
        end
    end
end
end