function S = flockFindRelLocS(Y,F);

% Find followers location relative to Y
% input Y.A, Y.B, Y.C
%       F list of followers [f index, X, Y;....]
% output S.o - indices of follower apporx on Y
%        S.l - indices of follower to the left of Y
%        S.r - indices of follower to the right of Y
% note: this is not 'real' left/right




% Allow some approx, say follower is less then 4 from Y, will be concidered
% on Y
% All followers distance
allDist = abs((Y.A*F(:,2)+Y.B*F(:,3)+Y.C)/sqrt((Y.A^2 + Y.B^2)));

yTol = 0.0001; 
S.o = F(find(allDist<yTol),:);
      
% Remove follower on Y
F = F(find(allDist>=yTol),:);

notOnY = Y.A * F(:,2) + Y.B * F(:,3) + Y.C * ones(length(F(:,2)),1) ; 
S.l = F(find(notOnY<0),:);
S.r = F(find(notOnY>0),:);

% S.o = o;
% S.l = l;
% S.r = r;

return



