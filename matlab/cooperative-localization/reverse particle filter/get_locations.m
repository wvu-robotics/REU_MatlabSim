function [particles,neighbors] = get_locations(ROBOTS,robot_source,range)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
neighbors = [];
numBots = length(ROBOTS);
particles = ROBOTS(robot_source).particles;
for L = 1:numBots % other robot
    
    d = ROBOTS(robot_source).laser(L);
    phi = ROBOTS(robot_source).bearing(L)+pi;
    
    %if the other robot is in detection / communication range
    %give it our dead-reckoning prediction of where it is
    %update our particle if we have one there already
    if d < range
        dx = d*cos(phi); %from r -> L
        dy = d*sin(phi); %from r -> L
        x0 = ROBOTS(L).position(1); %r pose
        y0 = ROBOTS(L).position(2); %r pose
        %give L, r's position of L, and mark that it has that
        %particle
        particles(:,L) = [x0+dx;y0+dy;1];
        neighbors = [neighbors,ROBOTS(L)];
    end
end


end

