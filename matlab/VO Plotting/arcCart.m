function [x,y] = arcCart(theta1, theta2, R, D)
        if theta2 > theta1
          theta = theta1:D:theta2;
        else
            theta = theta2:D:theta1;
        end 
          R = R*ones(1,length(theta));
          [x,y] = pol2cart(theta,R);
end