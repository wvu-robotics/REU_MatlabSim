function [dists,angles] = measure_dist_heading(ROBOTS,r)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

dists = [];
angles = [];
numBots = length(ROBOTS);

    for L = 1:numBots % other robot
        
        %distance from r to L
        d = norm(ROBOTS(L).position(1:2)- ROBOTS(r).position(1:2)); %truth
        d = d + normrnd(0,ROBOTS(r).sigmaRange,1,1); %noise
        %angle from r to L
        phi = atan2(ROBOTS(L).position(2)- ROBOTS(r).position(2), ROBOTS(L).position(1)- ROBOTS(r).position(1)); % truth
        phi = phi + angdiff(ROBOTS(r).position(3), ROBOTS(r).t_position(3)); %bias
        phi = phi + normrnd(0,ROBOTS(r).sigmaHeading,1,1); %noise
        dists = [dists, d];
        angles = [angles, phi];
    end


end

