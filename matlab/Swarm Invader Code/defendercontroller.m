function Velocity = defendercontroller(Robot)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
Velocity= [3,2];
S=.1;
Xi=Robot.invaders(1,1);
Yi=Robot.invaders(2,1);
Ui= Robot.invaders(3,1);
Vi=Robot.invaders(4,1);
Xrobot=Robot.position(1);
Yrobot= Robot.position(2);

[U,V,PSI11,PHI2] = objectFlow(Xi,Yi,Ui,Vi,S,Xrobot,Yrobot);
Velocity =[U,V]; 
end

