
% generate a 1D world with random robots values and have them assimilate


%world = zeros(10000,100);
%world(1,:) = rand(1,100);
world = rand(100,100);

figure()
hold on;

while true
    
   for t = 0:1000
       
    x = round(100*rand(1,1));
    y = round(100*rand(1,1));
    if x == 0
        x = 1;
    end
    
    if y == 0
        y =1;
    end
              
        if x == 1
            world(x,y) = world(x+1,y);     
        elseif x == 100
            world(x,y) = world(x-1,y);
        elseif y == 1
            world(x,y) = world(x,y+1);
        elseif y == 100
            world(x,y) = world(x,y-1);
        else
            if rand(1,1) > .5
                xn = x+1;
            else
                xn = x-1;
            end
            if rand(1,1) > .5
                yn = y+1;
            else
                yn = y-1;
            end
            world(x,y) = world(xn,yn);
            
        end
   end
        clf
        imagesc(world);
        
        hold on;
        pause(.0001);
end


imagesc(world)






