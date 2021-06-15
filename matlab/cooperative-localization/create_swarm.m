function ROBOTS = create_swarm(numBots,world_len,detection_rng)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

boids_count=numBots;
ROBOTS=Boid.empty;
Ks = 15;
Ka = 1;
Kc = 1;

for i=1:boids_count
   ROBOTS(i)=Boid(rand*world_len,rand*world_len, Ks,Ka, Kc,boids_count,i);
   ROBOTS(i).
end

end

