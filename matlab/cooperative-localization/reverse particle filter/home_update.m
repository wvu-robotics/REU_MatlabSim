function robot = home_update(robot,home_range)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
   
   home_dist = norm(robot.position - robot.home);
   if home_dist < home_range
        robot.covariance = [1, .01; .01, 1];
        robot.mean_position = robot.position;
        robot.particles = robot.particles .* 0;
   end


end

