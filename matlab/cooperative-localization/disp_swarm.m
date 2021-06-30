function disp_swarm(ROBOTS,range,show_detection_rng)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
numBots = length(ROBOTS);
KA = [];
KC = [];
KS = [];
KH = [];
KG = [];
        clf();
        %subplot(2,3,1);
    for r = 1:numBots
        plot(ROBOTS(r).position(1),ROBOTS(r).position(2),'ks');
        hold on;
        if show_detection_rng
            viscircles([ROBOTS(r).position(1),ROBOTS(r).position(2)],range);
            hold on;
        end
        if sum(ROBOTS(r).particles{1}(3,:)) > 1
            states = ROBOTS(r).particles{1}(1:2,ROBOTS(r).particles{1}(3,:) >.5);
            covars = ROBOTS(r).particles{2}(:,:,ROBOTS(r).particles{1}(3,:) > .5);
            %covar =  covar + cov(robot.particles{1}(1:2,robot.particles(3,:) >.5)');
           % [mean_pose,covar] = fusecovint(states,covars);
        else
            mean_pose = ROBOTS(r).mean_position;
            covar = ROBOTS(r).covariance;
        end
        
       % error_ellipse(covar, mean_pose)
        hold on;
        if sum(ROBOTS(r).color_particles) > 0
           COLOR= ROBOTS(r).color_particles./sum(ROBOTS(r).color_particles);
        else
            COLOR = [0,0,0];
        end
        if ROBOTS(r).is_beacon == 1
            plot(ROBOTS(r).mean_position(1), ROBOTS(r).mean_position(2), '^', 'color', COLOR);
            hold on;
        else
            plot(ROBOTS(r).mean_position(1), ROBOTS(r).mean_position(2), '*', 'color', COLOR);
            hold on;
        end
        plot(ROBOTS(r).goal(1), ROBOTS(r).goal(2), 'rx')
        
        KA = [KA,ROBOTS(r).Ka];
        KC = [KC,ROBOTS(r).Kc];
        KS = [KS,ROBOTS(r).Ks];
        KH = [KH,ROBOTS(r).Kh];
        KG = [KG,ROBOTS(r).Kg];
    end
    plot(ROBOTS(1).home(1),ROBOTS(1).home(2), 'bd','markersize',12)
    hold on;
    axis([-50 50 -50 50])
    
%     subplot(2,3,2)
%     histogram(KA)
%     title("alignment gain")
%     subplot(2,3,3)
%     histogram(KC)
%     title("cohesion gain")
%     subplot(2,3,4)
%     histogram(KS)
%     title("Seperation gain")
%     subplot(2,3,5)
%     histogram(KH)
%     title("Home gain")
%     subplot(2,3,6)
%     histogram(KG)
%     title("Goal gain")
    
    
    pause(.0001);
end

