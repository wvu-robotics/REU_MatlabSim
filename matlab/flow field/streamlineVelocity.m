function [u,v] = streamlineVelocity(x,y,Field)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
    u = 0;
    v = 0;
    
    [r,c] = size(Field);
    for i=1:1:r
        if Field(i,1) == 1
            [ui,vi,psii] = sourceFlow(Field(i,2),Field(i,3),Field(i,4),x,y);
            u = u + ui;
            v = v+vi;
        elseif Field(i,1) == 2
            [ui,vi,psii] = doubletFlow(Field(i,2),Field(i,3),Field(i,4),x,y);
            u = u + ui;
            v = v+vi;
        end
        
    end
    
    
end

