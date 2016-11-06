function result=quadcopter(motor1,motor2,motor3,motor4)
%QUADCOPTER Summary of this function goes here
%   Detailed explanation goes here

    global thetadot_quad;
    global theta_quad;
    global xdot_quad;
    global x_quad;
    global omega_quad;

    % Physical constants.
    g = 9.81;
    m = 0.5;
    L = 0.25;
    k = 3e-6;
    b = 1e-7;
    I = diag([5e-3, 5e-3, 10e-3]);
    kd = 0.25;
    
    dt = 0.005;

    in = [motor1; motor2; motor3; motor4];

    deviation = 00;
    thetadot_quad = deg2rad(2 * deviation * rand(3, 1) - deviation);
    
    omega_quad = thetadot2omega(thetadot_quad, theta_quad);
    a = acceleration(in, theta_quad, xdot_quad, m, g, k, kd);
    omegadot = angular_acceleration(in, omega_quad, I, L, b, k);

    % Advance system state.
    omega_quad = omega_quad + dt * omegadot;
    thetadot_quad = omega2thetadot(omega_quad, theta_quad); 
    theta_quad = theta_quad + dt * thetadot_quad;
    xdot_quad = xdot_quad + dt * a;
    x_quad = x_quad + dt * xdot_quad;
    
    %x_quad(3) = x_quad(3) -0.11 + 0.2*rand(1);
    
    result = [x_quad;theta_quad];
end


% Compute thrust given current inputs and thrust coefficient.
function T = thrust(inputs, k)
    T = [0; 0; k * sum(inputs)];
end

% Compute torques, given current inputs, length, drag coefficient, and thrust coefficient.
function tau = torques(inputs, L, b, k)
    tau = [
        L * k * (inputs(1) - inputs(3))
        L * k * (inputs(2) - inputs(4))
        b * (inputs(1) - inputs(2) + inputs(3) - inputs(4))
    ];
end

% Compute acceleration in inertial reference frame
% Parameters:
%   g: gravity acceleration
%   m: mass of quadcopter
%   k: thrust coefficient
%   kd: global drag coefficient
function a = acceleration(inputs, angles, vels, m, g, k, kd)
    gravity = [0; 0; -g];
    R = rotation(angles);
    T = R * thrust(inputs, k);
    Fd = -kd * vels;
    a = gravity + 1 / m * T + Fd;
end

% Compute angular acceleration in body frame
% Parameters:
%   I: inertia matrix
function omegad = angular_acceleration(inputs, omega, I, L, b, k)
    tau = torques(inputs, L, b, k);
    omegad = inv(I) * (tau - cross(omega, I * omega));
end

% Convert derivatives of roll, pitch, yaw to omega.
function omega = thetadot2omega(thetadot, angles)
    phi = angles(1);
    theta = angles(2);
    psi = angles(3);
    W = [
        1, 0, -sin(theta)
        0, cos(phi), cos(theta)*sin(phi)
        0, -sin(phi), cos(theta)*cos(phi)
    ];
    omega = W * thetadot;
end

% Convert omega to roll, pitch, yaw derivatives
function thetadot = omega2thetadot(omega, angles)
    phi = angles(1);
    theta = angles(2);
    psi = angles(3);
    W = [
        1, 0, -sin(theta)
        0, cos(phi), cos(theta)*sin(phi)
        0, -sin(phi), cos(theta)*cos(phi)
    ];
    thetadot = inv(W) * omega;
end

