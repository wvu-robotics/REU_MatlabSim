function [X_f,P_f] = Decentral_swarm_EKF(X_i, P_i, Y_m, F, dt, Q, R)

% This funcion runs a centralized extended kalman filter on some generated
% data taking input below and giving the output below 
% MODEL:
% x_k = f(x_k-1,u_k-1) + w_k-1
% y_k = h(x_k) + v_k
% ASSUMPTIONS:
% White Gaussian noise N(0,Q) for the estimation (w_k-1) ~ IMU
% White Gaussian noise N(0,R) for the measurement (v_k) ~ LIDAR
% INPUT:
% X_i is the current state matrix
% P_i is the current covariance matrix
% Y_m is the current measurement ~ LIDAR
% F is the kinematic input or gain to estimate the next state ~ IMU
% dt is the increment in time
% Q is the estimation noise covariance
% R is the measurement noise covariance
% OUTPUT:
% X_f is the a posteriori estimation for the state matrix
% P_f is the a posteriori estimation for the covariance matrix

number_of_robots = size(X_i,1)/3;
% Define the number of robots on th EKF
X_p = eye(number_of_robots*3)*X_i + eye(number_of_robots*3)*dt*F;
% X_p is the priori estimation for the state matrix
P_p = eye(number_of_robots*3)*P_i*eye(number_of_robots*3)' + Q;
% P_p is the priori estimation for the covariance matrix
Y_p = [];
% Expected measurement based on the a priori estimated state
H = zeros(number_of_robots,number_of_robots*3);
% Initialize the measurement matrix
for r = 1:number_of_robots % For each robot in the total number of ROBOTS
    dx = X_p(1:number_of_robots)- X_p(r);
    % Substract the x component from each robot to all x components of all the robots
    dy =  X_p(number_of_robots+1:2*number_of_robots) - X_p(r+number_of_robots);
    % Substract the y component from each robot to all y components of all the robots
    range = sqrt(dx.^2 + dy.^2);
    % Calculate the magnitude of this distance from each
    % robot to the rest of the robots
    for a = 1:number_of_robots % For each case in the total number of ROBOTS
       if range(a) == 0 % For the case that the robot has the range to itself
            dH_dx(a,1) = 0; % (Jacobbian of x) Set Jacobbian of x equal to 0
            dH_dy(a,1) = 0; % (Jacobbian of y) Set Jacobbian of y equal to 0
       else % For the case that the robot has the range to its neighbor
            dH_dx(a,1) = dx(a)/range(a); % (Jacobbian of x) Divide every distance x by every range (slope/rate of increment)
            dH_dy(a,1) = dy(a)/range(a); % (Jacobbian of y) Divide every distance y by every range (slope/rate of increment)
       end
    end
    dH_dtheta = zeros(size(dH_dx)); % (Jacobbian of theta) Set equal to 0 all the jacobbians for theta (only linear movement)
    H(:,r) = dH_dx; % Append jacobbian dx to H
    H(:,r+number_of_robots) = dH_dy; % Append jacobbian dy to H
    H(:,r+2*number_of_robots) = dH_dtheta; % Append jacobbian dtheta to H
    Y_p = [Y_p;range]; % Append distances to the expected measurement based on the a priori estimated state matrix
end

H0 = H;
for i = 2:number_of_robots
    H = [H;H0]; % Append the new measurment matrix to the previous measurment matrix
end

I = eye(3*number_of_robots);
% Identity matrix
residual = Y_m - Y_p;
% Residual = Current measurement - Expected measurement
S = H*P_p*(H')+R;
% Covariance error?
K = P_p*H'*inv(S);
% Kalman gain formula
X_f = X_p + K*residual;
% A posteriori estimation for the state matrix
P_f = (I-K*H)*P_p;
% A posteriori estimation for the covariance matrix
end
