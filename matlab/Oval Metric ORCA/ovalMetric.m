%% Oval Velocity Metric

%Description: Given a preferredVelocity and all acceptableVelocities, finds
%the 'distances' from each acceptableVelocity to the preferredVelocity.

%Shape Description: The metric uses consentric ovals around the
%preferredVelocity to measure distance. The ovals are squished
%translationally by transCost to incentivize motion orthogonal to the
%preferredVelocity over slowing down.

%Parameters:
%   transCost: A double that disincentivizes translation velocity change
%       transCost times more than orthogonal velocity change.
%   preferredVelocity: A 1x2 double that is the center of the metric
%       calculations
%   acceptableVelocities: An Nx2 double that holds an the ith velocity as
%       acceptableVelocity(i,:)

%Returns:
%   distances: An Nx1 double where the 'distance' between preferredVelocity
%       and acceptableVelocity(i,:) is distance(i);

%Remark: ovalMetric(1, preferredVelocity, acceptableVelocities) is the same
%   metric that unmodified ORCA uses, but this would be far less efficient.
function distances = ovalMetric(transCost, preferredVelocity, acceptableVelocities)
    %Finds the unit vector in the direction of the preferredVelocity
    transUnit = preferredVelocity ./ norm(preferredVelocity);
    
    %Finds the unit vector orthogonal to the preferredVelocity
    orthUnit = [-transUnit(2),transUnit(1)];
    
    %Calculates the distance^2 between the preferredVelocity and each
    %acceptable velocity
    distances = zeros(size(acceptableVelocities,1),1);
    for a = 1:size(acceptableVelocities,1)
        distances(a) = transCost.^2 * dot(acceptableVelocities(a,:) - preferredVelocity, transUnit).^2 + dot(acceptableVelocities(a,:) - preferredVelocity, orthUnit).^2;
    end
end

