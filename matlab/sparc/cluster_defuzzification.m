% State University of Campinas
% Evolving Fuzzy Control
% Function: cluster_defuzzification
% Description: Function that recursively calculate deffuzification of the
%              the system with input xk.
% Date: 30/10/2013 - Diego Domingos
% Inputs:
%        ld        : vector of local densities in k time
%        Q         : consequents vector
% Output:
%        G         : output of the system

function G=cluster_defuzzification(ld,Q)
    N=length(ld);
    % Assuming that G(xk) = sum(ld*Q)/sum(ld) = cons_ld/sum_ld
    sum_ld = 0;
    cons_ld = 0;
    for i=1:N,
        cons_ld = cons_ld + ld(i)*Q(i);
        sum_ld  = sum_ld + ld(i);
    end
    G=cons_ld/sum_ld;
end