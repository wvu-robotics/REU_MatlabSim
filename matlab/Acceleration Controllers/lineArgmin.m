%% Get Argmin on Line

%Description: Given the line with parametric equation u*s+v0, returns the
%point on that line minimumPoint which minimizes
%ovalMetric(k,pv,minimumPoint).

%Proof of Correctness: Acceleration Controller Formulation and Analysis in
%Google Drive

%See Also: Modified-ORCA/ovalMetric.m

%Parameters:
%   u: A 1x2 double parallel to the line
%   v0: A 1x2 double positioned on the line
%   pv: A 1x2 double for preferred velocity and the center of the
%       ovalMetric
%   k: A double for the translational cost of the ovalMetric

%Returns:
%   minimumPoint: A 1x2 double where ovalMetric(k,pv,minimumPoint) is
%       minimum
function minimumPoint = lineArgmin(u, v0, pv, k)
    %Finds the leftward orthogonal vector of pv
    orthPV = [-pv(2), pv(1)];
    
    %Sets the parametric parameter of the line s which corresponds to the
    %minimumPoint
    s = ( k^2 * dot(u,pv) * dot(pv-v0,pv) - dot(v0,orthPV) * dot(u,orthPV) )...
        / ( k^2 * dot(u,pv)^2 + dot(u,orthPV)^2 );
    
    minimumPoint = u * s + v0;
end

