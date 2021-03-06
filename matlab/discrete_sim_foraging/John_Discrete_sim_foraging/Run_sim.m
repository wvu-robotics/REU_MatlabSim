%script to start simulation
clc;
clear all;
close all;

init_density_food = 0.05;
init_num_bots = 1; %Making this too high breaks the placement functionality!
%init_home_loc = [10,10]; %home location in X-Y

%Set up simluation
world1 = World("What.png");
world1.gen_bots(init_num_bots);
world1.gen_food(init_density_food);
%world1.place_bots(10,10);

for i = 1:1
    disp("Starting Simulation!");
    figure();
    for ii=0:1000 %simulation ticks
        world1.tick();
        pause(.1);
    end

end