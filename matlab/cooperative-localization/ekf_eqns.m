function [X_f,P_f] = ekf_eqns(Y_m,Y_p,X_p,P_p, H, R)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here


residual = Y_m - Y_p;
S = H*P_p*(H')+R;
K = P_p*H'*inv(S);
X_f = X_p + K*residual;
I = eye(3*numBots);
P_f = (I-K*H)*P_p;

end

