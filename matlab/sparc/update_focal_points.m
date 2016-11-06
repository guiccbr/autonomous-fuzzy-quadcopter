% State University of Campinas
% Evolving Fuzzy Control
% Function: update_focal_points
% Description: Function that update the focal points
% Date: 10/11/2013 - Diego Domingos
% Inputs:
%        xk       : input vector in time k
%        xk_pre   : input vector in time k-1
%        yk       : output vector in time k
%        M        : vector of number of points in each cloud
%        Eps      : epsilon vector
%        Beta     : Beta vector
%        xf       : vector of focal points
% Output:
%        xf_      : focal points vector


function [xf_, M]=update_focal_points(xk, xk_pre, uk, uk_pre, M, Eps, Beta, E, B, xf, k)

    % Find associated data cloud Ni
    Ni=associated_cloud(xk, xk_pre, M, Eps, Beta);
    
    % Update number of points in cloud
    M(Ni) = M(Ni) + 1;
    
    % Generate inputs
    zk_pre = [xk_pre' uk_pre]';
    zk = [xk' uk]';
    zkf = [xf{Ni}(1:2)' uk]';
    
    % Calculate local density of input
    [ldx, trs1, trs2]=local_density(xk, xk_pre, M, Eps, Beta, Ni);
    
    % Calculate global density of input
    [gdx, Ek, Bk]=global_density(zk, zk_pre, k, E, B);
    
    % Calculate local density of focal point
    [ldf, trs1, trs2]=local_density(xf{Ni}(1:2), xk_pre, M, Eps, Beta, Ni);
    
    % Calculate global density of focal point
    [gdf, Ek, Bk]=global_density(zkf, zk_pre, k, E, B);
    
    % Evaluate the current input and update if necessary the focal point
    xf_ = xf;
    if ldx > ldf & gdx > gdf
        xf{Ni} = [xk;uk];
        xf_ = xf;
    end
    
end