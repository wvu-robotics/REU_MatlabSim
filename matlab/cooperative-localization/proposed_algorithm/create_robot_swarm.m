function ROBOTS = create_robot_swarm(numBots,world_len,detection_rng)
%creates an array of Robot objects
%   this class is used for swarm robots that follow boids rules for velocity control
%   numBots = number of robots to make
%   world_len = side length of world they are in
%   detection_rng = observable radius of the robot 
%   robots are based on the SMART2 robots

ROBOTS = Robot.empty();
gains = [.61102,9.9236,9.6478,3.2343,9.9235]; %bayesian optimized static gains
Ka = gains(1); % alignment gain
Ks = gains(2); % seperation gain
Kc = gains(3); % cohesion gain

for i=1:numBots
   %give them a random start location
   x = 2*rand*world_len - world_len; 
   y = 2*rand*world_len - world_len;
   ROBOTS(i)=Robot(x,y, Ks,Ka, Kc,numBots,i); % constructor
   ROBOTS(i).detection_range = detection_rng; % set dection range
   ROBOTS(i).covariance_d = [.1,  0,  0;    %give inital covarinace
                             0, .1,  0;
                             0, 0, .1];
   ROBOTS(i).covariance_e = ROBOTS(i).covariance_d;
   ROBOTS(i).P{i,i} = ROBOTS(i).covariance_e; %set the centralized covarinace matrix
   ROBOTS(i).ID = i;    %give the robot an ID
   ROBOTS(i).max_speed = .5; %set the max speed of the robot
end

end
