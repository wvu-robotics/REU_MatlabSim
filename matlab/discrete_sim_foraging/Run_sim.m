%script to start simulation
clc;
clear all;
close all;

init_density_food = 1;
init_num_bots = 80; %Making this too high breaks the placement functionality!
%init_home_loc = [10,10]; %home location in X-Y

%Set up simluation
world1 = World("world_image_1.png");
world1.gen_bots(init_num_bots);
world1.gen_food(init_density_food);
%world1.place_bots(35,35);

for i = 1:1
    disp("Starting Simulation!");
    for ii=0:100000 %simulation ticks
        world1.tick();
        pause(.01);
    end

end