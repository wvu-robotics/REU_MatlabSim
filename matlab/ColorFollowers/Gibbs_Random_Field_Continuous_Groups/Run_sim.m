%script to start simulation
clc;
clear;
close all;

timeStep = 2;
worldSize = 100;
init_num_bots = 30;

%spawnType = 'random';
% spawnType = 'singleCoordinated';
 spawnType = 'doubleCoordinated';
%spawnType = 'opposingGroups';

%Set up simluation
world1 = World(timeStep, worldSize, spawnType, init_num_bots);
world1.gen_bots();

for i = 1:1
    disp("Starting Simulation!");
    for ii=0:10000 %simulation ticks
        world1.tick();
        pause(.001);
    end

end