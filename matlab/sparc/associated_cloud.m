% State University of Campinas
% Evolving Fuzzy Control
% Function: associated_cloud
% Description: Function that find associated_cloud of input x
% Date: 08/11/2013 - Diego Domingos
% Inputs:
%        xk        : vector of inputs in time k
%        xk_pre   : vector of inputs in time k-1
%        M        : vector of Ms
%        Eps      : vector of Epsilon of time k-1
%        Beta     : vector Beta of time k-1
% Output:
%        Ni       : local density of cloud i in time k

function Ni=associated_cloud(xk, xk_pre, M, Eps, Beta)
    Ni=0;
    MaxLD = -1000;
    for j=1:length(M),
        [ld, e, b] = local_density(xk, xk_pre, M, Eps, Beta, j);
        if ld > MaxLD
            Ni = j;
        end
    end
end