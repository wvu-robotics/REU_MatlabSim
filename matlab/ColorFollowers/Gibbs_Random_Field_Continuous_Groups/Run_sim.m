%script to start simulation
clc;
clear;
close all;

timeStep = 2;
worldSize = 200;
init_num_bots = 50;

%spawnType = 'random';
% spawnType = 'singleCoordinated';
% spawnType = 'doubleCoordinated';
%spawnType = 'opposingGroups';
%spawnType = 'depot';
%spawnType = 'multiDepot';
spawnType = 'starDepot';

%Set up simluation
world1 = World(timeStep, worldSize, spawnType, init_num_bots);
world1.gen_bots();

for i = 1:1
    disp("Starting Simulation!");
    for ii=1:2000 %simulation ticks
        fprintf('Iteration %.0f\n', ii);
        world1.tick();
        pause(.01);
        
        
          F(ii) = getframe(gcf);
    end
 video = VideoWriter('MultiDepot1', 'MPEG-4');
 open(video);
 writeVideo(video, F);
 close(video)
end