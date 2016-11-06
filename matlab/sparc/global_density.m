% State University of Campinas
% Evolving Fuzzy Control
% Function: global_density
% Description: Function that recursively calculate the Global Density
%              of a cloud i.
% Date: 30/10/2013 - Diego Domingos
% Inputs:
%        zk       : vector of inputs in time k
%        zk_pre   : vector of inputs in time k-1
%        k        : time k
%        Ek_pre   : strange E in time k-1
%        Bk_pre   : B in time k-1
% Output:
%        gd       : global density of sample in time k
%        Ek       : updated E vector
%        Bk       : updated B vector

function [gd, Ek, Bk]=global_density(zk, zk_pre, k, Ek_pre, Bk_pre)
     Bk=Bk_pre + norm(zk_pre)^2;
     Ek=Ek_pre + zk_pre;
     Ak=zk'*Ek;
     gd=(k-1)/(k + (-1*(zk'*zk + 1)) -2*Ak + Bk);
end