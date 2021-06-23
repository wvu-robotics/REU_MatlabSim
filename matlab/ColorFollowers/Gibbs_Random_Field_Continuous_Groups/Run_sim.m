%script to start simulation
clc;
clear;
close all;

timeStep = 2;
%numColors = 1;
worldSize = 100;
init_num_bots = 80;

%spawnType = 'random';
% spawnType = 'singleCoordinated';
 spawnType = 'doubleCoordinated';
% spawnType = 'backAndForth';

%Set up simluation
world1 = World(timeStep, worldSize, spawnType, init_num_bots);
world1.gen_bots();

for i = 1:1
    disp("Starting Simulation!");
    for ii=0:100 %simulation ticks
        world1.tick();
        pause(.001);
    end

end