% State University of Campinas
% Evolving Fuzzy Control
% Function: local_density
% Description: Function that recursively calculate the Local Density
%              of a cloud i.
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

function [ld, Epsk, Betak]=local_density(xk, xk_pre, M, Eps, Beta, i)
    Epsk = Eps{i} + xk_pre;
    alpha = xk'*Epsk;
    Betak = Beta(i) + norm(xk_pre)^2;
    ld=(M(i))/(M(i)*(xk'*xk + 1) -2*alpha + Betak);
end