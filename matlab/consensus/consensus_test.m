% consensus algorithm test

% xi(t+1) = xi(t) + sum(Wij*(xj - xi))

len = 5;
world = zeros(len,len);

n = len^2;
di = 8;



X = 10*rand(n,1);

%% ---------------------- weight matrix ------------------------
W = zeros(n,n);

for x = 1:len
    for y = 1:len
        i = state_index(len,x,y);
        % bottom left--------------------------------------------
        if x == 1 && y ==1
            d = 3;
            w = 0;
            %to the right
            d = 5;
            j = state_index(len,x+1,y);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % up and right
            d = 8;
            j = state_index(len,x+1,y+1);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % up
            d = 5;
            j = state_index(len,x,y+1);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % center
            W(i,i) = 1-w;
        % bottom right-------------------------------------------
        elseif x ==len && y == 1
            d = 3;
            w = 0;
            %to the left
            d =5;
            j = state_index(len,x-1,y);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % up and left
            d = 8;
            j = state_index(len,x-1,y+1);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % up
            d = 5;
            j = state_index(len,x,y+1);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % center
            W(i,i) = 1-w;
        % top left-----------------------------------------------
         elseif x == 1 && y == len
            d = 3;
            w = 0;
            %to the right
            d = 5;
            j = state_index(len,x+1,y);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % down and right
            d = 8;
            j = state_index(len,x+1,y-1);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % down
            d = 5;
            j = state_index(len,x,y-1);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            %center
            W(i,i) = 1-w;
        % top right--------------------------------------------
        elseif x == len && y == len
            d = 3;
            w = 0;
            %to the left
            d = 5;
            j = state_index(len,x-1,y);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % down and left
            d = 8;
            j = state_index(len,x-1,y-1);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % down
            d = 5;
            j = state_index(len,x,y-1);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % center 
            W(i,i) = 1- w;
        %bottom wall-------------------------------------------
        elseif y ==1
            d = 5;
            w = 0;
            % left
            d = 5;
            j = state_index(len,x-1,y);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % up and left
            d = 8;
            j = state_index(len,x-1,y+1);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % up
            d = 8;
            j = state_index(len,x,y+1);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % up and right
            d = 8;
            j = state_index(len,x+1,y+1);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % right
            d = 5;
            j = state_index(len,x+1,y);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % center 
            W(i,i) = 1- w;
        % left wall-----------------------------------------
        elseif x == 1 
            d = 5;
            w = 0;
            % up
            d = 5;
            j = state_index(len,x,y+1);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % up and right
            d = 8;
            j = state_index(len,x+1,y+1);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % right
            d = 8;
            j = state_index(len,x+1,y);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % down and right
            d = 8;
            j = state_index(len,x+1,y-1);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % down
            d = 5;
            j = state_index(len,x,y-1);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % center 
            W(i,i) = 1- w;
            
        %right wall---------------------------------------
        elseif x == len
             d = 5;
             w = 0;
             % up
             d = 5;
            j = state_index(len,x,y+1);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % up and left
            d = 8;
            j = state_index(len,x-1,y+1);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % left
            d = 8;
            j = state_index(len,x-1,y);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
           % down and left
            d = 8;
            j = state_index(len,x-1,y-1);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % down
            d = 5;
            j = state_index(len,x,y-1);
            W(i,j) = 1/(d+1);  
            w = w + W(i,j);
            % center
            W(i,i) = 1-w;
            
        % top wall--------------------------------------
        elseif y == len
            d = 5;
            w = 0;
            % left
            d = 5;
            j = state_index(len,x-1,y);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % down and left
            d = 8;
            j = state_index(len,x-1,y-1);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % down
            d = 8;
            j = state_index(len,x,y-1);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % down and right
            d = 8;
            j = state_index(len,x+1,y-1);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % right
            d = 5;
            j = state_index(len,x+1,y);
            W(i,j) = 1/(d+1);
            w = w + W(i,j);
            % center
            W(i,i) = 1-w;
        % center ----------------------------------------
        else
            d = 8;
            w = 0;
            for u = x-1:x+1
                for v = y-1:y+1
                    j = state_index(len,u,v);
                    if i ~= j
                        W(i,j) = 1/(d+1);
                        w = w + W(i,j);
                    end
                end
            end
            W(i,i) = 1-w;
            %----------------------------------------------
        end
            
            
    end
end
mean(X)
%% ---------------------------------------itterate
figure()
imagesc(X);
HIST = X/n;
for t = 1:10
     Xn = zeros(size(X));
    for i = 1:n
        for j = 1:n
            
            Xn(i) = Xn(i) + W(i,j)*(X(j)-X(i));
            
        end
    end
    X = Xn;
    HIST = [HIST,X];
    clf;
    imagesc(reshape(X,[len,len]));
    pause(.1);
end
mean(X)


%%    functions -----------------------------------

 function s = state_index(len,x,y)
        s = (y-1)*len + x;
    end

