function robot = boids_update(robot,e_max,rho_max)
%UNTITLED9 Summary of this function goes here
%   Detailed explanation goes here

    % measure local density
    A = pi*robot.detection_range^2;
    rho = length(robot.neighbors) / A;
    covar = robot.covariance;
    % measure error in variance and dead reckoning mean error
    if sum(robot.particles{1}(3,:)) > 1
        states = robot.particles{1}(1:2,robot.particles{1}(3,:) >.5);
        covars = robot.particles{2}(:,:,robot.particles{1}(3,:) > .5);
        %covar =  covar + cov(robot.particles{1}(1:2,robot.particles(3,:) >.5)');
        [mean_pose,covar] = fusecovint(states,covars);
        robot.mean_position = mean_pose';
        robot.mean_covar = covar;
    else
        robot.mean_position = robot.position;
        robot.mean_covar = robot.covariance;
    end
    mean_error = norm(robot.position - robot.mean_position);
    
     % update boids parameters
    robot.max_speed = (norm(covar)+2)/(norm(robot.mean_position-robot.position)+1)^2;
    if robot.max_speed > 5
        robot.max_speed = 5;
    end
    robot.Ka = 0;%rho/rho_max + mean_error/e_max;
    robot.Kc = (norm(covar) + mean_error^2)/A; %norm([norm(robot.covariance), mean_error^2]);
    robot.Ks = 0;%A/(norm(covar) + mean_error^2);
    robot.Kh = (mean_error^2 + norm(covar))/(norm(robot.home-robot.mean_position)^2);
    robot.Kg = robot.detection_range/(norm(robot.goal-robot.mean_position)*norm(covar));
    
end

