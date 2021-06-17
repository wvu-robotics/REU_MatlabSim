function ROBOTS = create_swarm(numBots,world_len,detection_rng)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

boids_count=numBots;
ROBOTS=Boid.empty;
Ks = 15;
Ka = 1;
Kc = 1;

for i=1:boids_count
    x = rand*world_len;
    y = rand*world_len;
   ROBOTS(i)=Boid(x,y, Ks,Ka, Kc,boids_count,i);
   ROBOTS(i).detection_range = detection_rng;
   ROBOTS(i).covariance = [1, .01; .01, 1];
   ROBOTS(i).mean_position = [x,y];
end

end

