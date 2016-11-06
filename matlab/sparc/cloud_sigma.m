% State University of Campinas
% Evolving Fuzzy Control
% Function: cloud_sigma
% Description: Function that recursively calculate the Sigma
%              value.
% Date: 30/10/2013 - Diego Domingos
% Inputs:
%        x        : vector of inputs in time k-1
%        xk_pre   : vector of inputs in time k-1
%        Eps      : vector of Epsilon of time k-1
%        Beta     : vector Beta of time k-1
%        M        : time k
%        xf       : focal point of cloud i
%        i        : cloud i
% Output:
%        sig      : local density of cloud i in time k

function sig=cloud_sigma(x, xk_pre, Eps, Beta, M, xf, i)
    % Discover x points of cloud i
    cloud_x = {};
    for j=1:length(x),
        Ni=associated_cloud(x{j}, xk_pre, M, Eps, Beta);
        if Ni == i
            %cloud_x = cat(length(cloud_x)+1, cloud_x, x{j}); % was [cloud_x x(j)];
            cloud_x{length(cloud_x)+1}=x{j};
        end
    end
    
    % Sum of distances
    sum_dist = 0;
    for j=1:length(cloud_x),
        sum_dist = sum_dist + norm((xf{i}(1:2)-cloud_x{j}))^2;
    end
    
    % Sigma
    sig=sqrt(sum_dist/M(i));
end