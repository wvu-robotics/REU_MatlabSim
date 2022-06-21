% Thermal testing file for time-varying and location-varying thermal map
close all
clear
clc

%% Load sim law
addpath("Code of Laws");
SL = ThermalTestLaw();

%% Set up parameters
overallTime = 20; % s
dt = .05; % s 
steps = overallTime/dt; 

% Initialize thermals as a matrix of Thermals
thermalMap = ThermalMap(SL, 200, 0);

%% Set up video and figure
video = VideoWriter('Thermal Outputs/staticThermals.avi');
video.FrameRate = 1/dt;
open(video);

simFig = figure;

clf
xlim(SL.mapSize);
ylim(SL.mapSize);
daspect([1 1 1]);
colorbar;
cbLimits = [-1,SL.thermalStrengthMax];
colors = [6 42 127; 41 76 247; 102 59 231; 162 41 216; 222 24 200; 255 192 203] / 255;
x = [0:thermalMap.thermalPixels/(length(colors)-1):thermalMap.thermalPixels];
map = interp1(x/thermalMap.thermalPixels,colors,linspace(0,1,thermalMap.thermalPixels)); % Creates a color gradient for the map
colormap(map);
set(gca,'clim',cbLimits);

%% Run simulation
for step = 1:steps
    c1 = clock;
    %% Set up frame
    fprintf("Frame %g/%g\n",step,steps);
    hold on
        
    %% Render thermals
    finalThermalMap = thermalMap.renderThermals();

    thermalMapImg = imagesc(finalThermalMap,'XData',SL.mapSize,'YData',SL.mapSize);
    thermalMapImg.AlphaData = 1;
    hold off
    
    %% Save frame
    currFrame = getframe(simFig);
    writeVideo(video,currFrame);
    pause(0.0001);
    
    %% Step physics
    thermalMap.staticStep(dt);
    
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