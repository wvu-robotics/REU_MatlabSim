function robot = home_update(robot,home_range)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

home_dist = norm(robot.position(1:2) - robot.home);
if home_dist < home_range
    % update location
    robot.covariance = [.01,.001;.001,.01];
    robot.mean_covar = robot.covariance;
    robot.position = robot.t_position + normrnd(0,.001,1,3);
    robot.mean_position = robot.position;
   
    robot.particles{1} = robot.particles{1}*0;
    robot.particles{2} = robot.particles{2}*0;
   
    
    %get color particles
    if home_dist < home_range %within 5 squares
        theta = atan2d(robot.position(2) - robot.home(2),robot.position(1) - robot.home(1));
        if theta < -120 %red range
            robot.color_particles(1) = robot.color_particles(1) + 5;
        elseif theta < 0 %green
            robot.color_particles(2) = robot.color_particles(2) + 5;
        elseif theta < 120 %blue
            robot.color_particles(3) = robot.color_particles(3) + 5;
        else % red range
            robot.color_particles(1) = robot.color_particles(1) + 5;
        end
     
    end
end


end

