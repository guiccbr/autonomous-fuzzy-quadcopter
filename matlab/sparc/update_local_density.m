% State University of Campinas
% Evolving Fuzzy Control
% Function: update_local_density
% Description: Function that recursively update the local density vector.
% Date: 30/10/2013 - Diego Domingos
% Inputs:
%        xk       : vector of inputs in time k
%        xk_pre   : vector of inputs in time k-1
%        M        : vector of Ms
%        Eps      : vector of Epsilon of time k-1
%        Beta     : vector Beta of time k-1
% Output:
%        ld       : local density vector updated to time k
%        Eps      : updated Eps vector
%        Beta     : updated Beta vector

function [upd_ld, upd_Eps, upd_Beta]=update_local_density(xk, xk_pre, M, Eps, Beta)
    N=length(M);    
    upd_ld = [];
    upd_Eps = {};
    upd_Beta = [];
    for i=1:N,
        [ld_,Eps_,Beta_] = local_density(xk, xk_pre, M, Eps, Beta, i);
        upd_ld = [upd_ld ld_];
        upd_Eps{length(upd_Eps)+1} = Eps_; %was [upd_Eps Eps_];
        upd_Beta = [upd_Beta Beta_];    
    end
end