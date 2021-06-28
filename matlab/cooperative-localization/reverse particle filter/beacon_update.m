function ROBOTS = beacon_update(ROBOTS,ID, neighbors, cov_max)


%% become beacon

% if covariance if greater than threshold pause and become a beacon
%     and if there is no other beacon



%% stop becoming a beacon

% if neighbor's covariance is greater than mine it now becomes the beacon


%% implementation
found_beacon = 0;
beacon = [];
% if norm(ROBOTS(ID).mean_covar) < cov_max
%     ROBOTS(ID).is_beacon =0;
% end

if ROBOTS(ID).is_beacon == 1 && length(neighbors) <= 1
    ROBOTS(ID).time_as_beacon = ROBOTS(ID).time_as_beacon+1;
    found_beacon = 1;
    beacon = ID;
end

for r = neighbors
    if r.is_beacon == 1
        found_beacon =1;
        if length(neighbors) <= 1
            ROBOTS(r.ID).time_as_beacon = ROBOTS(r.ID).time_as_beacon+1;
        else
            ROBOTS(r.ID).time_as_beacon = 0;
        end
        beacon = r.ID;
        break;
    end
end

if found_beacon == 1
    for r = neighbors
        if norm(r.mean_covar) > norm(ROBOTS(beacon).mean_covar) %may need to flip the sign
            ROBOTS(beacon).is_beacon = 0;
            ROBOTS(beacon).time_as_beacon = 0;
            ROBOTS(r.ID).is_beacon = 1;
            ROBOTS(r.ID).time_as_beacon = 0;
            beacon = r.ID;
        end
    end
else
    if norm(ROBOTS(ID).mean_covar) > cov_max
        ROBOTS(ID).is_beacon =1;
        ROBOTS(ID).time_as_beacon = 0;
    end
 
end


end

