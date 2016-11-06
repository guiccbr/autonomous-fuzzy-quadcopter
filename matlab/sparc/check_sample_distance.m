% State University of Campinas
% Evolving Fuzzy Control
% Function: check_sample_distance
% Description: Function that check if a sample zk has good distance.
%              and return 1 if true and 0 if not.
% Date: 13/11/2013 - Diego Domingos
% Inputs:
%        x        : vector of inputs in time k
%        xk       : input vector in time k
%        xk_pre   : input vector in time k-1
%        M        : vector of number of points in each cloud
%        Eps      : epsilon vector
%        Beta     : Beta vector
%        xf       : vector of focal points
%        r        : vector of radius
% Output:
%        b        : boolean 1 or 0
%        rv       : updated radius vector

function [b, rv]=check_sample_distance(x, xk, xk_pre, Eps, Beta, M, xf, r)

    % Find associated cloud
    Ni=associated_cloud(xk, xk_pre, M, Eps, Beta);

    % Check distance of sample zk
    dk = norm(xk-xf{Ni}(1:2));
    
    b=1;
    % Compare it with all global density of focal points
    for i=1:length(xf),
        [ri, rv] = cloud_radius(x, xk_pre, Eps, Beta, M, xf, r, i);
        if(dk < ri/2)
            b=0;
        end
    end
end