%% Heading Velocity Metric

%Description: Given a preferredVelocity and all acceptableVelocities, finds
%the 'distances' from each acceptableVelocity to the preferredVelocity.

%Shape Description: Formally, the shape is Taxicab Metric in polar
%coordinates squished by transCon\st to disincentivize slowing down.
%   distances = headingDiff + transCost * magnitudeDiff;

%Parameters:
%   transCost: A double that disincentivizes translation velocity change
%       transCost times more than rotational change.
%   preferredVelocity: A 1x2 double that is the center of the metric
%       calculations
%   acceptableVelocities: An Nx2 double that holds an the ith velocity as
%       acceptableVelocity(i,:)

%Returns:
%   distances: An Nx1 double where the 'distance' between preferredVelocity
%       and acceptableVelocity(i,:) is distance(i);
function distances = headingMetric(transCost, preferredVelocity, acceptableVelocities)
   %Finds the difference in headings between acceptable and preferred
   %velocities
   headingDiff = zeros(size(acceptableVelocities,1),1);
   for a = 1:size(acceptableVelocities,1)
       if norm(acceptableVelocities(a,:)) > 0 && norm(preferredVelocity) > 0
           headingDiff(a) = acos( dot(preferredVelocity, acceptableVelocities(a,:)) / ( norm(preferredVelocity) * norm(acceptableVelocities(a,:)) ) );
       else
           headingDiff(a) = pi;
       end
   end
   
   magnitudeDiff = abs(norm(preferredVelocity) - vecnorm(acceptableVelocities, 2, 2));
   
   distances = headingDiff + transCost * magnitudeDiff;
end

