% 1 D road builder

env = zeros(5,100,3); % robots, variance, isBeacon
rng = 5;
env(:,rng+1,1) = 1;
env(1,1,3) = 1;


figure()
imagesc(env(:,:,1));
pause(.1)
for t = 1: 100
     for r = 2:5
        for x = rng:100-rng
       
            if env(r,x,1) == 1 % is a robot
                %---------------------------------- communicate
                for i = x-rng:x+rng
                    for j = 1:5
                        if env(j,i,3) == 1 % if found a beacon
                            env(r,x,2) = 0; %my variance is the variance of the beacon
                        end
                    end
                end
                %-----------------------move ?
                if env(r,x,3) ~= 1
                    env(r,x,2) = env(r,x,2) + .1;
                end
                if env(r,x,2) < .5 && env(r,x,3) ~= 1
                    env(r,x+1,:) = env(r,x,:);
                    env(r,x,:) = env(r,x,:)*0;
                    
                else
                    env(r,x,3) = 1;
                end
                
                break;
            end
            
        end
        
    end
    imagesc(env(:,:,1));
    pause(.01)
end



