function robot = boids_gain(robot,W)
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
     d_h = norm(robot.home-robot.mean_position);
     d_g = norm(robot.goal-robot.mean_position);
     gains = W*[rho; mean_error; norm(covar); d_h; d_g];
     
    robot.max_speed = gains(1);
    if robot.max_speed > 2
        robot.max_speed = 2;
    end
    robot.Ka = gains(2);
    robot.Kc = gains(3);
    robot.Ks = gains(4);
    robot.Kh = gains(5);
    robot.Kg = gains(6);
    
end

