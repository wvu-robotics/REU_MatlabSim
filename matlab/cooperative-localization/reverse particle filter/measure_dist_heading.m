function [dists,angles] = measure_dist_heading(ROBOTS,r,noise)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

dists = [];
angles = [];
numBots = length(ROBOTS);

    for L = 1:numBots % other robot
        
        %distance from r to L
        d = norm(ROBOTS(L).position(1:2)- ROBOTS(r).position(1:2))+ normrnd(0,noise,1,1);
        %angle from r to L
        phi = atan2(ROBOTS(L).position(2)- ROBOTS(r).position(2), ROBOTS(L).position(1)- ROBOTS(r).position(1))+ normrnd(0,noise,1,1);
        
        dists = [dists, d];
        angles = [angles, phi];
    end


end

