% State University of Campinas
% Evolving Fuzzy Control
% Function: fuzzy_system
% Description: Function that do fuzzy system
% Date: 13/11/2013 - Diego Domingos

function u=fuzzy_system(e,de, r, yk)
    % Global vectors
    global x;        % vector of inputs in all time
    global ld;       % local density_vector
    global ld_focal; % local density vector of focal points
    global gd_focal; % global density vector of focal points
    global xf;       % focal points
    global Eps;      % epsilon vector of local density
    global Beta;     % beta vector of local density
    global E;        % E vector of global density
    global B;        % B vector of global density
    global Lambda;   % lambda vector
    global Q;        % consequent_vector
    global gd;       % global density
    global C;        % offline constant
    global M;        % number of points in cluster vector
    global xk_pre;   % x in time k-1
    global zk_pre;   % z in time k-1
    global uk;       % u in time k
    global uk_pre;   % u in time k-1
    global k;        % time k
    global rv;       % vector of radius
    global ref_pre;  % previus reference
    
    %disp('Starting fuzzy system...');
    xk=[e;de];
    k=k+1;
    x{length(x)+1}=xk;
    
    % Update consequents
    Q=update_consequent(Q, Lambda, r, yk, C, uk);
    
    % Compute the local density for the current input
    [ld, Eps, Beta]=update_local_density(xk, xk_pre, M, Eps, Beta);
    
    Lambda = update_lambda(ld);
    
    % Generate control signal and update Z point
    uk=cluster_defuzzification(ld,Q);
    zk_pre = [xk_pre' uk_pre]';
    zk = [xk' uk]';

    gen_check = 0;
    dis_check = 0;
    
    % Conditions (15) and (16) both satisfied?
    [gen_check, E, B] = check_sample_generalization(zk, zk_pre, k, E, B, xf);
    
    % Debug only

        % Yes
        if gen_check == 1
            % Check if is distante enough to create another data cloud
            [dis_check, rv] = check_sample_distance(x, xk, xk_pre, Eps, Beta, M, xf, rv);
            if dis_check == 1
                xf{length(xf)+1} = [xk;uk];
                Q = [Q uk];
                M=[M 1];
                rv=[rv 1];
                Eps=[Eps 0];
                Beta=[Beta 0];
                disp(strcat(int2str(length(M)),'o new data cloud created!'));
            end
        end
        % Else, only update focal points if necessary    
        if gen_check == 0 || dis_check == 0
            % Update focal points
            %disp('Updating focal points...');
            [xf, M] = update_focal_points(xk, xk_pre, uk, uk_pre, M, Eps, Beta, E, B, xf, k);
        end
    %disp(length(M));
    u=uk;
    %disp(u);
    ref_pre = r;
end