function centroid = flockBaryCenter(F);

% This simple function utilazes polygeom
% to find the barycenter of set of 2D points
% [centroidX centroidY]  = baryCenter([X1 Y1; X2 Y2;...])
% First it will make a convex hull from the vertexes given
% only then it will aply polygeom

tmp = size(F);
if (tmp(2) == 3) % for indexed F = [n,X,Y ]
    % convet the XY matrix to [Xi],[Yi]
    X = F(:,2);
    Y = F(:,3);
elseif  (tmp(2) == 2) % for simple F = [X,Y ]
    X = F(:,1);
    Y = F(:,2);
else
    error( ' Bad input ');
end

% Find a order that will make a convex hull polygon
k = convhull(X,Y);
% Rearrange the vector in such way
X = X(k);
Y = Y(k);
% Now, finally find the barycenter
polyProps = polygeom(X,Y);
centroid = [polyProps(2) polyProps(3)];
return
