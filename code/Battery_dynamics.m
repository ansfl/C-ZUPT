function [I_t, V_t, V_oc, SoC] = Battery_dynamics( tt, P_out )
% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : NA                                                 %
%   Output : Battery dynamics based on 4S LiPo pack             %
%                                                               %
% -------------------------- Content -------------------------- %

% ----------- Battery Parameters ---------- %
C_bat = 3.0*3600;                           % [Coulombs] = 3000 mAh x 3600 s = A*s
C_1 = 2.5;                                  % [F]
R_0 = 0.04;                                 % [Ohm]
R_1 = 0.05;                                 % [Ohm]
V_oc_f = 14.0;                              % [V]
v_1 = 4.8;                                  % [V] 
v_2 = -2.0;                                 % [V]

% Time span and initial conditions
Dt = tt(2)-tt(1);                           % Resolution (numerical DAE)

% Interpolation of INPUT data (u(t) NOT x(t))
P_fun = @(t) interp1(tt, P_out, t, 'pchip');  % or 'spline'

% ------------ Initial Guesses ------------ %
SoC_0 = 1.0;                                % Fully charged
V_10 = 0;                                   % Initial polarization voltage
V_oc0 = V_oc_f + v_1*SoC_0 + v_2*SoC_0^2;   % Initial open-circuit voltage

% NOTE : Initial P_fun(t=0) = Dt !!!
I_0 = P_fun(Dt)/(V_oc0 - R_0*(P_fun(Dt)/V_oc0)); % Initial current guess
V_0 = V_oc0 - R_0*I_0 - V_10;               % Terminal voltage from model

X_0 = [I_0; V_0; V_oc0; SoC_0; V_10];       % Initial states
dX_0 = [0; 0; 0; 0; 0];                     % Initial derivatives

% DAE system                                % X = [I, V, V_oc, SoC, V_1];
F_x = @(t, x, dx) [                         % F(t, x, dx) = 0
    % ---------------- X(t) --------------- %
    x(1) - P_fun(t)/x(2);                   % I(t)
    x(2) - x(3) + R_0*x(1) + x(5);          % V(t) 
    x(3) - (V_oc_f + v_1*x(4) + v_2*x(4)^2);% V_oc(t)
    % --------------- dX/dt --------------- %
    dx(4) + x(1)/C_bat;                     % SoC'
    dx(5) + x(5)/(R_1*C_1) - x(1)/C_1;      % V_1'
];

%
opts = odeset('RelTol',1e-6,'AbsTol',1e-8); % [y0_mod, dy0_mod] = decic(F_x, t_0, X_0, [0 0 0 1 0], dX_0, [0 0 0 0 0]);


[~, Y] = ode15i(F_x, tt, X_0, dX_0, opts); % Solve the system

[I_t, V_t, V_oc, SoC] = deal(Y(:,1)', Y(:,2)', Y(:,3)', Y(:,4)');