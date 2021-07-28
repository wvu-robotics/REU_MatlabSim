
clc
clear
close all

global CURRENT_KEY_PRESSED 
CURRENT_KEY_PRESSED = '';
H = figure;
set(H,'KeyPressFcn',@buttonPress);

numberOfAgents = 2;
env = agentEnv(numberOfAgents,@ORCAController,2,.01);

env.setGoalPositions(zeros(numberOfAgents,2));

env.agents(1).setUpPublisher('/turtle5/cmd_vel');
env.agents(1).setUpSubscriber('/vicon/turtle5/turtle5');
env.agents(2).setUpPublisher('/turtle6/cmd_vel');
env.agents(2).setUpSubscriber('/vicon/turtle6/turtle6');

counter = 0;
env.realTime = true;
env.pathVisibility(false);

while true
   counter = counter + 1;
   env.tickRos;
end

function buttonPress(src,event)
  global CURRENT_KEY_PRESSED
  CURRENT_KEY_PRESSED = event.Key;
end
