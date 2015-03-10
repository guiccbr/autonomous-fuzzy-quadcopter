% State University of Campinas
% Evolving Fuzzy Control
% Function: check_sample_generalization
% Description: Function that check if a sample zk has good generalization.
%              and return 1 if true and 0 if not.
% Date: 13/11/2013 - Diego Domingos
% Inputs:
%        zk       : sample in time k
%        zk_pre   : sample in time k-1
%        k        : time k
%        Ek       : E vector
%        Bk       : B vector
%        xf       : vector of focal points
% Output:
%        b        : boolean 1 or 0

function [b, Ek_, Bk_]=check_sample_generalization(zk, zk_pre, k, Ek, Bk, xf)

    % Check global density of sample zk
    [gd, Ek_, Bk_] = global_density(zk, zk_pre, k, Ek, Bk);
    
    b=1;
    % Compare it with all global density of focal points
    for i=1:length(xf),
        if(gd < global_density(xf{i}, zk_pre, k, Ek, Bk))
            b=0;
        end
    end
end