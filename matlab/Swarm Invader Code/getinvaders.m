function Invaders = getinvaders(Robots, Neighbors)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
Invaders = Robots.invaders;

for r=Neighbors
    if r.Isinvader== 1
        newinvader = 1;
        for i= Invaders
            if i==r.ID
                newinvader=0;
                %TODO update position of old invader
            end
        end
        if newinvader == 1    
        Invaders = [Invaders,[r.position(1);r.position(2);r.velocity(1);r.velocity(2);1]];
        end
    end
end


end

