function disp_swarm(ROBOTS,range,show_detection_rng)

numBots = length(ROBOTS);
KA = [];
KC = [];
KS = [];
KH = [];
KG = [];
        clf();
        %subplot(2,3,1);
    for r = 1:numBots
        % plot truth data-----------------------------------------
        quiver(ROBOTS(r).position_t(1),ROBOTS(r).position_t(2),ROBOTS(r).velocity_t(1), ROBOTS(r).velocity_t(2),'k');
        hold on;
        plot(ROBOTS(r).position_t(1), ROBOTS(r).position_t(2), 'ks');
        hold on;
        if show_detection_rng
            viscircles([ROBOTS(r).position_t(1),ROBOTS(r).position_t(2)],range);
            hold on;
        end
        %plot estimated position and color-------------------------------------------
        error_ellipse(ROBOTS(r).mean_covar, [ROBOTS(r).mean_position(1), ROBOTS(r).mean_position(2)])
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
        % plot dead reckoning position---------------------------------------------------------
        plot(ROBOTS(r).position(1), ROBOTS(r).position(2), 'bo');
        hold on;
        quiver(ROBOTS(r).position(1), ROBOTS(r).position(2),ROBOTS(r).velocity(1), ROBOTS(r).velocity(2), 'b');
        hold on;
        
        %plot goal position---------------------------------------------
        plot(ROBOTS(r).goal(1), ROBOTS(r).goal(2), 'rx')
        
        KA = [KA,ROBOTS(r).Ka];
        KC = [KC,ROBOTS(r).Kc];
        KS = [KS,ROBOTS(r).Ks];
        KH = [KH,ROBOTS(r).Kh];
        KG = [KG,ROBOTS(r).Kg];
    end
    %plot home------------------------------------------------------
    plot(ROBOTS(1).home(1),ROBOTS(1).home(2), 'bd','markersize',12)
    hold on;
    viscircles([ROBOTS(1).home(1),ROBOTS(1).home(2)],range);
    hold on;
    title("Square = truth, * = estimate, o = dead reckoning");
    axis([-50 50 -50 50])
    
    %plot gain distributions
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

