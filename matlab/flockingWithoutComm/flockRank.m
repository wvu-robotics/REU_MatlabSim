function k = flockRank(me,A)

% inputs:   Sx sub group of followers locations
%           me - current follower index
% 

% Rank of 'me' in Sx equals to this location 
% since Sx is already sorted
k = find(A(:,1) == me);

% this codition below will never happen since the 
% ranking is done within the follower's own group
if isempty(k)
    k=9999;
end
return 
