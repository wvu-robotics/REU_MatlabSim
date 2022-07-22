%% Prepare workspace
close all
clear
clc

%% I'm stuff 0_0
% This will send Adam Pooley a notification
url = "https://api.pushover.net/1/messages.json";
appToken = "a5q8yhpz3t9c7pzzmrmzczpejcyozy";
ACPToken = "uti2e2jd1jhtujgbtnaaop95rubtmq";
message = "Simulation Complete.";
webwrite(url,'token',appToken,'user',ACPToken,'message',message);