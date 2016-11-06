% State University of Campinas
% Evolving Fuzzy Control
% Function: membership_degree
% Description: Function that recursively calculate degree of membership
%              of a input xk in cloud i.
% Date: 30/10/2013 - Diego Domingos
% Inputs:
%        xk       : vector of inputs in time k
%        xk_pre   : vector of inputs in time k-1
%        M        : vector of Ms
%        Eps      : vector of Epsilon of time k-1
%        Beta     : vector Beta of time k-1
%        i        : cloud i
% Output:
%        ld       : local density of cloud i in time k
%        Epsk     : updated Eps
%        Betak    : updated Beta

function mean_ld=membership_degree(xk, xk_pre, M, Eps, Beta, i)
N=length(M);
ld_i=local_density(xk, xk_pre, M, Eps, Beta, i);
sum_ld=0;
for j=1:N,
    sum_ld=sum_ld + local_density(xk, xk_pre, M, Eps, Beta, j)
end
mean_ld = ld_i/sum_ld;
end