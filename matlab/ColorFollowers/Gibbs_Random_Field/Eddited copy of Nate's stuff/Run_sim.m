%script to start simulation
clc;
clear all;
close all;

timeStep = 1;
numColors = 3;
init_num_bots = 25; %Making this too high breaks the placement functionality!
%init_home_loc = [10,10]; %home location in X-Y

%Set up simluation
world1 = World("world_image_1.png", timeStep);
world1.gen_bots(init_num_bots, numColors);
%world1.gen_food(init_density_food);
%world1.place_bots(35,35);

for i = 1:1
    disp("Starting Simulation!");
    for ii=0:1000 %simulation ticks
        world1.tick();
        pause(.001);
    end

end