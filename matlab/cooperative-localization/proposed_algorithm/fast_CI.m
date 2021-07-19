function [fused_state,fused_covariance] = fast_CI(states,covars)
%states = [l,n] matrix :  holds all the states to be fused where
%                         l is the number of states
%                         n is the number of measured states to fuse
%
%covars = [l,l,n] matrix: holds all the covariances of the states to be
%                         fused where Z axis is number of measured states


[len,neigh] = size(states); % get size of states and number of fusions
inv_covars = 0*covars;      % inverse of all the given covariance matricies
sum_info = zeros(len,len);  % total sum of the inverse of the covariance matricies


for n = 1:neigh
   inv_covars(:,:,n) = inv(covars(:,:,n));
   sum_info = sum_info + inv_covars(:,:,n);
   
end

sum_except_info = 0;        % ask Chizhao
for n = 1:neigh
   sum_except_info = sum_except_info + det(inv_covars(:,:,n)) - det(sum_info - inv_covars(:,:,n)); 
end

omega = zeros(1,neigh);          % weight per state and covariance
sum_inv_covar = zeros(len,len);  % weighted sum of the inverse covariances
sum_state = zeros(len,1);        % weighted sum of the state vectors

for n = 1:neigh
    omega(n) = (  det(sum_info) ...
                - det(sum_info-inv_covars(:,:,n))...
                + det(inv_covars(:,:,n))         )...
              /(neigh*det(sum_info) + sum_except_info);
                
   sum_inv_covar = sum_inv_covar + omega(n)*inv_covars(:,:,n);
   sum_state = sum_state + omega(n)*inv_covars(:,:,n)*states(:,n);
end

fused_covariance = inv(sum_inv_covar);      % final fused covars 
fused_state = fused_covariance * sum_state; % final fused state


end

