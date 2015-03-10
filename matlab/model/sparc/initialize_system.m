function initialize_system()
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
    global gd;       % global_density_vector
    global C;        % offline constant
    global M;        % number of points in cluster vector
    global xk_pre;   % x in time k-1
    global zk_pre;   % z in time k-1
    global uk;       % u in time k
    global uk_pre;   % u in time k-1
    global k;        % time k
    global rv;        % vector of radius
    global ref_pre;  % previus refeference

    
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
    global yaw_gd;       % global_density_vector
    global yaw_C;        % offline constant
    global yaw_M;        % number of points in cluster vector
    global yaw_xk_pre;   % x in time k-1
    global yaw_zk_pre;   % z in time k-1
    global yaw_uk;       % u in time k
    global yaw_uk_pre;   % u in time k-1
    global yaw_k;        % time k
    global yaw_rv;        % vector of radius
    global yaw_ref_pre;  % previus refeference
    
    %N=cat(length(N)+1,N,[a;b]);
    xk=[2;2];
    x={xk};
    ld=[1];
    ld_focal=[0];
    gd_focal=[0];
    xf={[2 2 1]'};
    Eps={[0;0]};
    Beta=[0];
    E=0;
    B=0;
    Lambda=[0];
    Q=[0];
    C=1000; %800000
    M=[1];
    xk_pre=[0;0];
    zk_pre=[xk' 0]';
    uk=400000;
    uk_pre=0;
    k=1;
    rv=[1];
    ref_pre=0;
    
    yaw_xk=[2;2];
    yaw_x={yaw_xk};
    yaw_ld=[1];
    yaw_ld_focal=[0];
    yaw_gd_focal=[0];
    yaw_xf={[2 2 1]'};
    yaw_Eps={[0;0]};
    yaw_Beta=[0];
    yaw_E=0;
    yaw_B=0;
    yaw_Lambda=[0];
    yaw_Q=[0];
    yaw_C=100;
    yaw_M=[1];
    yaw_xk_pre=[0;0];
    yaw_zk_pre=[xk' 0]';
    yaw_uk=0;
    yaw_uk_pre=0;
    yaw_k=1;
    yaw_rv=[1];
    yaw_ref_pre=0;
end