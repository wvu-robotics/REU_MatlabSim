function Velocity = defendercontroller(Robot)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
Velocity= [3,2];
S=.1;
%Do we want Xi,Yi,etc to represent the closest invaders coordinates?
%iterate through Robot.neigbors looking for a neighbor that has a isEnemy
%flag set to true. This means that that neighbor boid is really an enemy
%and we should store the coordinates of the cloests enemyn neighbor here to
%Xi yadda yadda. This will have the effect of making us choose the closests
%enemy -Nate

%Handle to hold the closest invader object
 closest_enemy = Robot.empty;
 smallest_dist = 2;

 for i=1:size(Robot.neighbors)
     if(Robot.neighbors(i).isInvader == 1)
        %Use robot.neighbors(i).position to figure out distance from you
        %If it's the closest enemy you've seen so far then set
        %closest_enemy = Robot.neighbors(i) -Nate
        dist = norm(Robot.neighborts(i).position - Robot.position);
        if dist < smallest_dist
            smallest_dist = dist;
            closest_enemy = Robot.neighbors(i);
        end
     end
 end
%Down here, extract Xi, Yi, yadda yadda from cloest_enemy
if ~isempty(closest_enemy)
    Xi=closest_enemy(1,1);

% Xi2=Robot.invaders(1,1);
Yi=closest_enemy(2,1);
% Yi2=Robot.invaders(3,1);
Ui= closest_enemy(3,1);
% Ui2= Robot.invaders(2,1);
Vi=closest_enemy(4,1);
% Vi2= Robot.invaders(4,1);
Xrobot=Robot.position(1);
% Xrobot2=Robot.position(2);
Yrobot= Robot.position(2);
% Yrobot2= Robot.position(1);

[U,V,PSI11,PHI2] = objectFlow(Xi,Yi,Ui,Vi,S,Xrobot,Yrobot);
% [U,V,PSI11,PHI2] = objectFlow(Xi2,Yi2,Ui2,Vi2,S,Xrobot2,Yrobot2);
Velocity =[U,V];
end
end

