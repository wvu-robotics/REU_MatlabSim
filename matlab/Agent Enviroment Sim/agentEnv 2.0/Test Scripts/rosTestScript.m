
clc
clear
close all
global CURRENT_KEY_PRESSED 
CURRENT_KEY_PRESSED = '';
H = figure;
set(H,'KeyPressFcn',@buttonPress);
rosinit('10.255.96.126');
numberOfAgents = 1;
env = agentEnv(numberOfAgents,@rosController,10,.1);
env.setAgentPositions(zeros(numberOfAgents, 2));
env.setGoalPositions([5, 5]);
env.agents(1).setUpPublisher('/turtle1/cmd_vel/');
env.agents(1).setUpSubscriber('/turtle1/cmd_vel/');
counter = 0;
ENV.realTime = true;
while true 
   counter = counter + 1;
   env.tickRos;
end
rosshutdown;

function buttonPress(src,event)
  global CURRENT_KEY_PRESSED
  CURRENT_KEY_PRESSED = event.Key;
end