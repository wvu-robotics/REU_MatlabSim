% Main script: loads parameter variables and runs swarm step function
%% Clear
close all
clear
clc

videoName = ("ThermalVisual.avi");

%% Setup video and figure
%Create video
dt = 0.02;
video = VideoWriter(videoName);
video.FrameRate = 1/dt;
open(video);

%Create frame
simFig = figure('Visible','on');

%% Prepare data
mapSize = [-1000,1000];
thermalRes = 100;
thermalPos = [0,0];
thermalRad = 800;
thermalMaxStrength = 10;

numCycles = 1;
cycleFadeTime = 4; %s, time to individually fade in or out
cycleUpTime = 4; %s
cycleDownTime = 2;

cycleTotal = 2*cycleFadeTime + cycleUpTime + cycleDownTime;
steps = numCycles * cycleTotal / dt;

[X,Y,mapStrength] = getMapStrength(thermalMaxStrength, mapSize, thermalRes, thermalPos, thermalRad);
minStr = min(min(mapStrength));
maxStr = max(max(mapStrength));

surface = NaN;

%% Run simulation
for step = 1:steps
    fprintf("Step %g/%g\n",step,steps);
    
    time = step*dt;
    relTime = mod(time,cycleTotal);
    
    if(relTime < cycleFadeTime)
        thermalStrength = interp1([0,cycleFadeTime],[0,thermalMaxStrength],relTime,'linear');
    elseif(relTime < cycleFadeTime + cycleUpTime)
        thermalStrength = thermalMaxStrength;
    elseif(relTime < 2*cycleFadeTime + cycleUpTime)
        thermalStrength = interp1([cycleFadeTime + cycleUpTime,2*cycleFadeTime + cycleUpTime],[thermalMaxStrength,0],relTime,'linear');
    else
        thermalStrength = 0;
    end
    
    [X,Y,mapStrength] = getMapStrength(thermalStrength, mapSize, thermalRes, thermalPos, thermalRad);
    color = mapStrength;
    if(class(surface) == "double")
        surface = surf(X,Y,mapStrength,color);
    end
    surface.ZData = mapStrength;
    surface.CData = mapStrength;
    colorbar;
    
    zlim([minStr, maxStr]);
    caxis([minStr, maxStr]);

    xlabel("X-Position (m)");
    ylabel("Y-Position (m)");
    zlabel("Updraft Velocity (m/s)");

    % Save video frame
    currFrame = getframe(simFig);
    writeVideo(video,currFrame);
end

close(video);

function [X,Y,mapStrength] = getMapStrength(currStrength, mapSize, thermalRes, thermalPos, thermalRad)
    side = linspace(mapSize(1),mapSize(2),thermalRes);
    [X,Y] = meshgrid(side,side);

    distTherm = sqrt((X-thermalPos(1)).^2 + (Y-thermalPos(2)).^2);
    mapStrength = currStrength .* exp(-(3*distTherm/thermalRad).^2) .* (1-(3*distTherm/thermalRad).^2);
end