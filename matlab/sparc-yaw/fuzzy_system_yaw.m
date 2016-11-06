% State University of Campinas
% Evolving Fuzzy Control
% Function: fuzzy_system
% Description: Function that do fuzzy system
% Date: 13/11/2013 - Diego Domingos

function u=fuzzy_system_yaw(e,de, r, yk)
    % Global vectors
    global yaw_x;        % vector of inputs in all time
    global yaw_ld;       % local density_vector
    global yaw_ld_focal; % local density vector of focal points
    global yaw_gd_focal; % global density vector of focal points
    global yaw_xf;       % focal points
    global yaw_Eps;      % epsilon vector of local density
    global yaw_Beta;     % beta vector of local density
    global yaw_E;        % E vector of global density
    global yaw_B;        % B vector of global density
    global yaw_Lambda;   % lambda vector
    global yaw_Q;        % consequent_vector
    global yaw_gd;       % global density
    global yaw_C;        % offline constant
    global yaw_M;        % number of points in cluster vector
    global yaw_xk_pre;   % x in time k-1
    global yaw_zk_pre;   % z in time k-1
    global yaw_uk;       % u in time k
    global yaw_uk_pre;   % u in time k-1
    global yaw_k;        % time k
    global yaw_rv;       % vector of radius
    global yaw_ref_pre;  % previus reference
    
    %disp('Starting fuzzy system...');
    xk=[e;de];
    yaw_k=yaw_k+1;
    yaw_x{length(yaw_x)+1}=xk;
    
    % Update consequents
    yaw_Q=update_consequent(yaw_Q, yaw_Lambda, r, yk, yaw_C, yaw_uk_pre);
    
    % Compute the local density for the current input
    [yaw_ld, yaw_Eps, yaw_Beta]=update_local_density(xk, yaw_xk_pre, yaw_M, yaw_Eps, yaw_Beta);
    
    yaw_Lambda = update_lambda(yaw_ld);
    
    % Generate control signal and update Z point
    yaw_uk=cluster_defuzzification(yaw_ld,yaw_Q);
    yaw_zk_pre = [yaw_xk_pre' yaw_uk_pre]';
    zk = [xk' yaw_uk]';

    gen_check = 0;
    dis_check = 0;
    
    % Conditions (15) and (16) both satisfied?
    [gen_check, yaw_E, yaw_B] = check_sample_generalization(zk, yaw_zk_pre, yaw_k, yaw_E, yaw_B, yaw_xf);
    
    % Debug only

        % Yes
        if gen_check == 1
            % Check if is distante enough to create another data cloud
            [dis_check, yaw_rv] = check_sample_distance(yaw_x, xk, yaw_xk_pre, yaw_Eps, yaw_Beta, yaw_M, yaw_xf, yaw_rv);
            if dis_check == 1
                yaw_xf{length(yaw_xf)+1} = [xk;yaw_uk];
                yaw_Q = [yaw_Q yaw_uk];
                yaw_M=[yaw_M 1];
                yaw_rv=[yaw_rv 1];
                yaw_Eps=[yaw_Eps 0];
                yaw_Beta=[yaw_Beta 0];
                disp(strcat(int2str(length(yaw_M)),'o new data cloud created!'));
            end
        end
        % Else, only update focal points if necessary    
        if gen_check == 0 || dis_check == 0
            % Update focal points
            %disp('Updating focal points...');
            [yaw_xf, yaw_M] = update_focal_points(xk, yaw_xk_pre, yaw_uk, yaw_uk_pre, yaw_M, yaw_Eps, yaw_Beta, yaw_E, yaw_B, yaw_xf, yaw_k);
        end
    %disp(length(M));
    u=yaw_uk;
    %disp(u);
    yaw_ref_pre = r;
end