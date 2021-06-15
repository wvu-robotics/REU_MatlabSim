function disp_swarm(ROBOTS)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
numBots = length(ROBOTS);
clf()
    for r = 1:numBots
        plot(ROBOTS(r).position(1),ROBOTS(r).position(2),'ks');
        %viscircles([ROBOTS(r).pose(1),ROBOTS(r).pose(2)],range);
        hold on;
        x_mean = mean(ROBOTS(r).particles(1,ROBOTS(r).particles(3,:) >.5));
        y_mean = mean(ROBOTS(r).particles(2,ROBOTS(r).particles(3,:) >.5));

        covar =  cov(ROBOTS(r).particles(1:2,ROBOTS(r).particles(3,:) >.5)');
        error_ellipse(covar, [x_mean,y_mean])
        hold on;
        plot(x_mean, y_mean, 'go')
        hold on;
    end
    axis([-50 50 -50 50])
    pause(.0001);
end

