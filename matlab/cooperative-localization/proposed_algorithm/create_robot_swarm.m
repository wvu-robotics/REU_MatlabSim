function ROBOTS = create_robot_swarm(numBots,world_len,detection_rng)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here


ROBOTS = Robot.empty();
gains = [.61102,9.9236,9.6478,3.2343,9.9235];
Ka = gains(1);
Ks = gains(2);
Kc = gains(3);

for i=1:numBots
    x = 2*rand*world_len - world_len;
    y = 2*rand*world_len - world_len;
   ROBOTS(i)=Robot(x,y, Ks,Ka, Kc,numBots,i);
   ROBOTS(i).detection_range = detection_rng;
   ROBOTS(i).covariance_d = [.1,  0,  0; 
                             0, .1,  0;
                             0, 0, .1];
   ROBOTS(i).covariance_e = ROBOTS(i).covariance_d;
   ROBOTS(i).P{i,i} = ROBOTS(i).covariance_e;
   ROBOTS(i).ID = i;
   ROBOTS(i).max_speed = .5;
end

end
