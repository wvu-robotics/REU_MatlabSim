function [X_f,P_f] = imu_encoder_vel_EKF(X_i,P_i,y_m,u,Q,R,t)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% 
% F = [1, 0, -u(1)*t*sin(u(3)*t+X_i(3));
%      0, 1, u(2)*t*cos(u(3)*t+X_i(3));
%      0, 0, 1];
 
F = eye(3);
 V_i = norm(X_i(1:2));
 
 H = [X_i(1)/V_i, X_i(2)/V_i, 0;
        0,          0,        0];
 
I = eye(3);
X_p = X_i + u*t;%.*[cos(u(3)*t+X_i(3));sin(u(3)*t+X_i(3)); 1];
P_p = F*P_i*F' + Q;
V_p = norm(X_p(1:2));
y_p = [V_p;u(3)];

residual = y_m - y_p;
S = H*P_p*H'+R;
K = P_p*H'*inv(S);
X_f = X_p + K*residual;
P_f = (I-K*H)*P_p;



end

