clc
clear
close all
global CURRENT_KEY_PRESSED % this takes in current key pressed on the
                           % keyboard to allow control of the invader's
                           % movements in whichever direction the user
                           % chooses.
CURRENT_KEY_PRESSED = '';
H = figure; % generates the figure the user must click into before
            % pressing any keys to move the invader. the figure will have
            % no valuable information, if any, and the user simply needs to
            % click the figure then attemtpt to press keys for manual
            % control. 
set(H,'KeyPressFcn',@buttonPress);
rosinit('192.168.10.134'); % IP address of the ROS Master
numberOfAgents = 1; % Manually controlled agents. this would be one invader
env = agentEnv(numberOfAgents,@rosController,10,.1); 
% creates a separate environment class apart from ENV for manual control. 
env.setAgentPositions(zeros(numberOfAgents, 2));
env.setGoalPositions([5, 5]);
env.agents(1).setUpPublisher('/cmd_vel/'); % publishing to cmd_vel topic
% env.agents(1).setUpSubscriber('/cmd_vel/');
counter = 0;
ENV.realTime = true;
while true 
   counter = counter + 1;
   env.tickRos;
end
rosshutdown; % this should not execute due to the while loop.
% if it does, this means the while loop is not working.

function buttonPress(src,event)
  global CURRENT_KEY_PRESSED
  CURRENT_KEY_PRESSED = event.Key;
end