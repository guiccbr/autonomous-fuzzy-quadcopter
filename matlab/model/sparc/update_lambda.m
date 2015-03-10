% State University of Campinas
% Evolving Fuzzy Control
% Function: update_lambda
% Description: Function that updates lambda vector
% Date: 30/10/2013 - Diego Domingos
% Inputs:
%        ld        : local density vector
% Output:
%        L         : updated membership vector

function L=update_lambda(ld)
    L = [];
    % calculate the sum of all ld
    sum_ld = 0;
    for i=1:length(ld),
        sum_ld = sum_ld + ld(i);
    end
    
    % membership calc
    for i=1:length(ld),
        L=[L ld(i)/sum_ld];
    end
end