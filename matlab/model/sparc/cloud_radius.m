% State University of Campinas
% Evolving Fuzzy Control
% Function: cloud_radius
% Description: Function that calculate the radius of a cloud.
% Date: 08/11/2013 - Diego Domingos
% Inputs:
%        x        : vector of inputs in time k-1
%        xk_pre   : vector of inputs in time k-1
%        Eps      : vector of Epsilon of time k-1
%        Beta     : vector Beta of time k-1
%        M        : time k
%        xf       : strange E in time k-1
%        r        : radius  vector
%        i        : cloud i
% Output:
%        ri       : local density of cloud i in time k
%        r_       : updated radius vector

function [ri, r_]=cloud_radius(x, xk_pre, Eps, Beta, M, xf, r, i)
    ri = 0.5*r(i) + 0.5*cloud_sigma(x, xk_pre, Eps, Beta, M, xf, i);
    r(i) = ri;
    r_ = r;
end