
clc
clear
close all
global CURRENT_KEY_PRESSED 
CURRENT_KEY_PRESSED = '';
H = figure;
set(H,'KeyPressFcn',@buttonPress);
rosinit('192.168.10.133');
numberOfAgents = 1;
env = agentEnv(numberOfAgents,@rosController,2,.01);
env.agents(1).setShape(.25*[-1,-1; -1, 1; 1,0]);
env.setAgentPositions(zeros(numberOfAgents, 2));
env.setGoalPositions([5, 5]);
env.agents(1).setUpPublisher('/cmd_vel');
env.agents(1).setUpSubscriber('/vicon/turtle1/turtle1');
counter = 0;
env.realTime = true;
env.pathVisibility(false);
while true 
   counter = counter + 1;
   env.tickRos;
end
rosshutdown;

function buttonPress(src,event)
  global CURRENT_KEY_PRESSED
  CURRENT_KEY_PRESSED = event.Key;
end