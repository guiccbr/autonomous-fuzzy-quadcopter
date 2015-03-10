% State University of Campinas
% Evolving Fuzzy Control
% Function: consequent_reward
% Description: Function that calculate the consequent reward or penalty
% Date: 04/11/2013 - Diego Domingos
% Inputs:
%        ld_pre   : vector of local density in k-1 time
%        yk       : output in k time
%        ref_pre  : reference in k-1 time
%        C        : offline defined normalized constant
%        i        : cloud i
% Output:
%        delta_Q  : reward/penalty to Q

function delta_Q=consequent_reward(Lambda, ref_pre, yk, C,i, uk_pre)

    umax = 410000;   % was 0.005
    umin = 407000;
    
    delta_Q = C*Lambda(i)*(ref_pre - yk);

    
    %if delta_Q > umax
    %    delta_Q = umax;
    %elseif delta_Q < umin
    %    delta_Q = umin;
    %end
    
    %disp(uk_pre);
    %disp(delta_Q);
    
    if uk_pre<umin && delta_Q < 0
        delta_Q = 0;
        %disp('Delta_Q = 0');
    elseif uk_pre > umax && delta_Q > 0
        delta_Q = 0;
        %disp('Delta_Q = 0');
    end
    

    %disp('Delta Q:');
    %disp(delta_Q);
end