function disp_swarm(ROBOTS,range,show_detection_rng)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
numBots = length(ROBOTS);
clf()

    for r = 1:numBots
        plot(ROBOTS(r).position(1),ROBOTS(r).position(2),'ks');
        if show_detection_rng
            viscircles([ROBOTS(r).position(1),ROBOTS(r).position(2)],range);
            hold on;
        end
        
        error_ellipse(ROBOTS(r).covariance, ROBOTS(r).mean_position)
        hold on;
        plot(ROBOTS(r).mean_position(1), ROBOTS(r).mean_position(2), 'go')
        hold on;
    end
    plot(ROBOTS(1).home(1),ROBOTS(1).home(2), 'bd','markersize',12)
    hold on;
    %axis([-50 50 -50 50])
    pause(.0001);
end

