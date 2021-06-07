function [n X Y] = flockStep(Ptn, Flwr, Ldr, me);

% flockStep is the main loop the follower execute
% input: Ptn  = 3 by n, desired pattern [i,x,y;i+1,x,y;...]
%        Flwr = 3 by n, current follower location
%        Ldr  = 2 by 1, leader loacation
%        me   = index of me in Flwr(:,1)
% Output: [X Y] - Where should follow go

% Find flock barycenter XY
B = flockBaryCenter(Flwr);
% Find the line leader--barycenter
Y = flockSetY(Ldr,B);
% Separate flock relative to Y. on/left/right
S = flockFindRelLocS(Y,Flwr);

% Start looking at Ptn
% Rotate Ptn and move near Ldr
PtnF = flockAlignPnY(Ptn,Y,Ldr,B);
% Find final baricenter
Bf = flockBaryCenter(PtnF);
F = flockFindRelLocS(Y,PtnF);

%A = flockSort(A,L,B)
S.o = flockSort(S.o,Ldr,B);
S.l = flockSort(S.l,Ldr,B);
S.r = flockSort(S.r,Ldr,B);

F.o = flockSort(F.o,Ldr,Bf);
F.l = flockSort(F.l,Ldr,Bf);
F.r = flockSort(F.r,Ldr,Bf);

if ~isempty(find(S.o(:,1)==me))
    % Case me on S.o, approx baricenter
    k = flockRank(me,S.o);
    if (k<=flockNumOfBots(F.o))
        n = F.o(k,1);
        X = F.o(k,2);
        Y = F.o(k,3);
        return
    elseif (flockNumOfBots(S.l) <= flockNumOfBots(S.r))
        i = flockNumOfBots(F.l) - flockNumOfBots(S.l) + k - flockNumOfBots(F.o);
        n = F.l(i,1);
        X = F.l(i,2);
        Y = F.l(i,3);
        return
    else
        i = flockNumOfBots(F.r) - flockNumOfBots(S.r) + k - flockNumOfBots(F.o);
        n = F.r(i,1);
        X = F.r(i,2);
        Y = F.r(i,3);
        return
    end
    % Case me on S.o - END
elseif ~isempty(find(S.r(:,1)==me))
    % Case me on S.r, relative right
    k = flockRank(me,S.r);
    if (k<=flockNumOfBots(F.r))
        n = F.r(k,1);
        X = F.r(k,2);
        Y = F.r(k,3);
        return
    elseif (k <= (flockNumOfBots(F.r) + flockNumOfBots(F.o) - flockNumOfBots(S.o)))
        H = [ S.l(flockNumOfBots(F.l)+1:end,:) ; ...
            S.r(flockNumOfBots(F.r)+1:end,:) ];
        H = flockSort(H,Ldr,B);
        kk = flockRank(me,H);
        p = (kk + flockNumOfBots(S.o));
        n = F.o(p,1);
        X = F.o(p,2);
        Y = F.o(p,3);
        return
    else
        i = flockNumOfBots(F.l) - (k - flockNumOfBots(F.r) - flockNumOfBots(F.o) + flockNumOfBots(S.o)) + 1;
        n = F.l(i,1);
        X = F.l(i,2);
        Y = F.l(i,3);
        return
    end
    % Case me on S.r - END
elseif ~isempty(find(S.l(:,1)==me)) % So, I got to be in S.l
    % Case me on S.l, relative left
    k = flockRank(me,S.l);
    if (k<=flockNumOfBots(F.l))
        n = F.l(k,1);
        X = F.l(k,2);
        Y = F.l(k,3);
        return
    elseif (k <= (flockNumOfBots(F.l) + flockNumOfBots(F.o) - flockNumOfBots(S.o)))
        H = [ S.l(flockNumOfBots(F.l)+1:end,:) ; ...
            S.r(flockNumOfBots(F.r)+1:end,:) ];
        H = flockSort(H,Ldr,B);
        kk = flockRank(me,H);
        p = (kk + flockNumOfBots(S.o));
        n = F.o(p,1);
        X = F.o(p,2);
        Y = F.o(p,3);
        return
    else
        i = flockNumOfBots(F.r) - (k - flockNumOfBots(F.l) - flockNumOfBots(F.o) + flockNumOfBots(S.o)) + 1;
        n = F.r(i,1);
        X = F.r(i,2);
        Y = F.r(i,3);
        return
    end
    % Case me on S.l - END
else
    error(' Something is really wrong, sorry. flockStep. ');
end





