%% gradientVector: Find Gradient Vector
%
% Description: Given the preferred velocity pv, the curent velocity v, and
% the translational cost k, this finds the gradient vector on
% ovalMetric(k, pv, minimumPoint).
%
% Proof of Correctness: Acceleration Controller Formulation and Analysis in
% Google Drive
%
% Parameters:
%   pv: A 1x2 double describing the preferred velocity
%   v: A 1x2 double describing the current velocity
%   k: A double describing how costly translational changes in velocity are
%
% Returns:
%   gradVector: A 1x2 double which points in the direction that increases
%       the oval metric return value fastest.
%
% Precondition:
%   norm(pv) > 0
%
% See Also: Modified-ORCA/ovalMetric.m
function gradVector = gradientVector(pv, v, k)
    orthPV = [-pv(2),pv(1)];
    
    gradVector(1) = dot(v - pv, k^2*pv(1)*pv - pv(2)*orthPV);
    gradVector(2) = dot(v - pv, k^2*pv(2)*pv + pv(1)*orthPV);
    
    gradVector = 2 * gradVector ./ (norm(pv)^2);
end

