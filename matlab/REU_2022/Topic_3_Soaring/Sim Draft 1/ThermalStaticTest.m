% Thermal testing file for time-varying and location-varying thermal map
close all
clear
clc

%% Load sim law
addpath("Code of Laws");
SL = ThermalTestLaw();

%% Set up parameters
overallTime = 30; % s
dt = .05; % s 
steps = overallTime/dt; 

% Initialize thermals as a matrix of Thermals
thermalMap = ThermalMap(SL);

%% Set up video and figure
video = VideoWriter('Thermal Outputs/staticThermals.avi');
video.FrameRate = 1/dt;
open(video);

simFig = figure;

%% Initialize map background
clf
xlim(SL.mapSize);
ylim(SL.mapSize);
daspect([1 1 1]);

%Setup colorbar
colorbar;
cbLimits = [-1,SL.thermalStrengthMax];
set(gca,'clim',cbLimits);

%Setup colormap color-scheme
xColor = linspace(0,SL.thermalPixels,length(SL.CMColors)) / SL.thermalPixels;
map = interp1(xColor,SL.CMColors,linspace(0,1,SL.thermalPixels)); % Creates a color gradient for the map
colormap(map);

%% Run simulation
for step = 1:steps
    c1 = clock;
    %% Set up frame
    fprintf("Frame %g/%g\n",step,steps);
        
    %% Render thermals
    thermalMap.renderThermals();
    
    %% Save frame
    currFrame = getframe(simFig);
    writeVideo(video,currFrame);
    pause(0.0001);
    
    %% Step physics
    thermalMap.staticStep();
    
    %% Find and print elapsed time
    c2 = clock;
    elapsedTime = c2(6)-c1(6);
    % If minute advances, elapsedTime will appear negative (1min20sec - 0min50sec = 20sec-50sec = -30sec)
    if(elapsedTime < 0) 
        elapsedTime = elapsedTime + 60;
    end
    fprintf("%g sec\n",elapsedTime);
end

close(video);