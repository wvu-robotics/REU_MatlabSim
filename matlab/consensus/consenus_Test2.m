%consensus test 2
N = 25;
X = rand(N,1)+5;

phi = zeros(n,n);

for x = 1:len
    for y = 1:len
        i = state_index(len,x,y);
        % bottom left--------------------------------------------
        if x == 1 && y ==1
            
            w = 0;
            %to the right
            j = state_index(len,x+1,y);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % up and right
            
            j = state_index(len,x+1,y+1);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % up
            
            j = state_index(len,x,y+1);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % center
            phi(i,i) = -w;
        % bottom right-------------------------------------------
        elseif x ==len && y == 1
            
            w = 0;
            %to the left
            
            j = state_index(len,x-1,y);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % up and left
            
            j = state_index(len,x-1,y+1);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % up
            
            j = state_index(len,x,y+1);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % center
            phi(i,i) = -w;
        % top left-----------------------------------------------
         elseif x == 1 && y == len
            d = 3;
            w = 0;
            %to the right
            d = 5;
            j = state_index(len,x+1,y);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % down and right
            d = 8;
            j = state_index(len,x+1,y-1);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % down
            d = 5;
            j = state_index(len,x,y-1);
            phi(i,j) = 1;
            w = w + phi(i,j);
            %center
            phi(i,i) = -w;
        % top right--------------------------------------------
        elseif x == len && y == len
            d = 3;
            w = 0;
            %to the left
            d = 5;
            j = state_index(len,x-1,y);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % down and left
            d = 8;
            j = state_index(len,x-1,y-1);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % down
            d = 5;
            j = state_index(len,x,y-1);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % center 
            phi(i,i) = - w;
        %bottom wall-------------------------------------------
        elseif y ==1
            d = 5;
            w = 0;
            % left
            d = 5;
            j = state_index(len,x-1,y);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % up and left
            d = 8;
            j = state_index(len,x-1,y+1);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % up
            d = 8;
            j = state_index(len,x,y+1);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % up and right
            d = 8;
            j = state_index(len,x+1,y+1);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % right
            d = 5;
            j = state_index(len,x+1,y);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % center 
            phi(i,i) = - w;
        % left wall-----------------------------------------
        elseif x == 1 
            d = 5;
            w = 0;
            % up
            d = 5;
            j = state_index(len,x,y+1);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % up and right
            d = 8;
            j = state_index(len,x+1,y+1);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % right
            d = 8;
            j = state_index(len,x+1,y);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % down and right
            d = 8;
            j = state_index(len,x+1,y-1);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % down
            d = 5;
            j = state_index(len,x,y-1);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % center 
            phi(i,i) = - w;
            
        %right wall---------------------------------------
        elseif x == len
             d = 5;
             w = 0;
             % up
             d = 5;
            j = state_index(len,x,y+1);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % up and left
            d = 8;
            j = state_index(len,x-1,y+1);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % left
            d = 8;
            j = state_index(len,x-1,y);
            phi(i,j) = 1;
            w = w + phi(i,j);
           % down and left
            d = 8;
            j = state_index(len,x-1,y-1);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % down
            d = 5;
            j = state_index(len,x,y-1);
            phi(i,j) = 1;  
            w = w + phi(i,j);
            % center
            phi(i,i) = -w;
            
        % top wall--------------------------------------
        elseif y == len
            d = 5;
            w = 0;
            % left
            d = 5;
            j = state_index(len,x-1,y);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % down and left
            d = 8;
            j = state_index(len,x-1,y-1);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % down
            d = 8;
            j = state_index(len,x,y-1);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % down and right
            d = 8;
            j = state_index(len,x+1,y-1);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % right
            d = 5;
            j = state_index(len,x+1,y);
            phi(i,j) = 1;
            w = w + phi(i,j);
            % center
            phi(i,i) = -w;
        % center ----------------------------------------
        else
            d = 8;
            w = 0;
            for u = x-1:x+1
                for v = y-1:y+1
                    j = state_index(len,u,v);
                    if i ~= j
                        phi(i,j) = 1;
                        w = w + phi(i,j);
                    end
                end
            end
            phi(i,i) = -w;
            %----------------------------------------------
        end
            
            
    end
end

% G(x) = avg X

[V,M] = eig(phi);
mu = M*ones(length(W),1);
rho_upperbound = -2./(mu*n);


rho = 1/max(abs(diag(phi)));


W = eye(n) + rho*phi;


[V,LAM] = eig(W);

lam = LAM*ones(length(W),1);

%H(z) = (1+c)*z^-1/(1+c*z^-2) : 0<c< 1
mean(X)
figure()
imagesc(X);
HIST = X/n;
for t = 1:1
     Xn = zeros(size(X));
    for i = 1:n
        for j = 1:n
            
            Xn(i) = Xn(i) + W(i,j)*(X(i));
            
        end
    end
    X = Xn;
    HIST = [HIST,X];
    clf;
    imagesc(HIST);
    pause(.1);
end
mean(X)



%%    functions -----------------------------------

 function s = state_index(len,x,y)
        s = (y-1)*len + x;
    end

