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

% Initialize background of map
thermalPixels = 200;
mapX = linspace(SL.mapSize(1),SL.mapSize(2),thermalPixels);
mapY = linspace(SL.mapSize(1),SL.mapSize(2),thermalPixels);
mapDiff = (SL.mapSize(2)-SL.mapSize(1))/(thermalPixels-1);

% Initialize thermals as a matrix of Thermals
thermalMap = ThermalMap(SL, 0);

%% Set up video and figure
video = VideoWriter('thermals1.avi');
video.FrameRate = 1/dt;
open(video);

simFig = figure;

%% Run simulation
for step = 1:steps
    c1 = clock;
    %% Set up frame
    fprintf("Frame %g/%g\n",step,steps);
    clf
    hold on
    xlim(SL.mapSize);
    ylim(SL.mapSize);
    daspect([1 1 1]);
    colorbar;
    cbLimits = [-1,SL.thermalStrengthMax];
    %colors = [6 42 127; 11 84 254; 41 76 247; 71 67 239; 102 59 231; 132 50 223; 162 41 216; 192 32 208; 222 24 200; 252 15 192; 255 192 203] / 255;
    colors = [6 42 127; 41 76 247; 102 59 231; 162 41 216; 222 24 200; 255 192 203] / 255;
    x = [0:thermalPixels/(length(colors)-1):thermalPixels];
    map = interp1(x/thermalPixels,colors,linspace(0,1,thermalPixels)); % Creates a color gradient for the map
    colormap(map);
    set(gca,'clim',cbLimits);
    
    %% Render thermals
    finalThermalMap = thermalMap.renderThermals(thermalPixels, mapX, mapY, mapDiff);

    thermalMapImg = imagesc(finalThermalMap,'XData',SL.mapSize,'YData',SL.mapSize);
    thermalMapImg.AlphaData = 1;
    hold off
    
    %% Save frame
    currFrame = getframe(simFig);
    writeVideo(video,currFrame);
    pause(0.0001);
    
    %% Step physics
    thermalMap.staticStep(dt);
    
        % Find and print elapsed time
    c2 = clock;
    elapsedTime = c2(6)-c1(6);
    % If minute advances, elapsedTime will appear negative (1min20sec - 0min50sec = 20sec-50sec = -30sec)
    if(elapsedTime < 0) 
        elapsedTime = elapsedTime + 60;
    end
    fprintf("%g sec\n",elapsedTime);
end

close(video);