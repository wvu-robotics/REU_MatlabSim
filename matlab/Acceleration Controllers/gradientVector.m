%% Find Gradient Vector

%Description: Given the preferred velocity pv, the curent velocity v, and
%the translational cost k, this finds the gradient vector on the oval
%metric.

%Proof of Correctness: Acceleration Controller Formulation and Analysis in
%Google Drive

%Parameters:
%   pv: A 1x2 double describing the preferred velocity
%   v: A 1x2 double describing the current velocity
%   k: A double describing how costly translational changes in velocity are

%Returns:
%   gradientVector: A 1x2 double which points in the direction that
%       increases the oval metric return value fastest.
function gradientVector = gradientVector(pv,v,k)
    orthPV = [-pv(2),pv(1)];
    
    gradientVector(1) = dot(v - pv, k^2*pv(1)*pv - pv(2)*orthPV);
    gradientVector(2) = dot(v - pv, k^2*pv(2)*pv + pv(1)*orthPV);
    
    gradientVector = 2 * gradientVector ./ (norm(pv)^2);
end

