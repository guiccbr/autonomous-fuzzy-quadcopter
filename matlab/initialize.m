global thetadot_quad;
global theta_quad;
global xdot_quad;
global x_quad;
global omega_quad;

thetadot_quad = zeros(3,1);
theta_quad = zeros(3,1);
xdot_quad = zeros(3,1);
x_quad = zeros(3,1);
omega_quad = zeros(4,1);

addpath(genpath('sparc'));

addpath(genpath('sparc-yaw'));

initialize_system