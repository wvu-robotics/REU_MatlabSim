function A = flockSort(A,L,B)

% input A(2,:) XY
%   L - leader XY
%   Barycenter XY
% output - A sorted be dist from L and from B

% Dist from lead (L)
A(:,4) = sqrt((A(:,2)-L(1)).^2 + (A(:,3)-L(2)).^2);
% dist from Barycenter
A(:,5) = sqrt((A(:,2)-B(1)).^2 + (A(:,3)-B(2)).^2);
% sort row decending
A = sortrows(A,[-4 -5]);
A = A(:,1:3);
return
