% State University of Campinas
% Evolving Fuzzy Control
% Function: input_error
% Description: Function that calculate the error of input
% Date: 09/11/2013 - Diego Domingos
% inputs:
%        r    : reference signal
%        z    : output of model
% outputs:
%        e    : error between r and z
%        de   : change in error

function [e, de]=input_error(r, z)
    e=r-z;
    de=e;
end
