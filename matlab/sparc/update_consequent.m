% State University of Campinas
% Evolving Fuzzy Control
% Function: update_consequent
% Description: Function that update the consequent vector
% Date: 04/11/2013 - Diego Domingos
% Inputs:
%        Q        : consequent vector
%        ld_pre   : vector of local density in k-1 time
%        yk       : output in k time
%        ref_pre  : reference in k-1 time
%        C        : offline defined normalized constant
%        i        : cloud i
% Output:
%        Q        : update consequent vector

function Q=update_consequent(Q, Lambda, ref_pre, yk, C, uk_pre)
    for i=1:length(Lambda),
        Q(i) = Q(i) + consequent_reward(Lambda, ref_pre, yk, C, i, uk_pre);
    end
end