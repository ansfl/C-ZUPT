function [m, L, g, J_xx, J_yy, J_zz, k_T, k_M, eta_eff, Mat_Mix, w_hover, T_max] = sys_params()
% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : N/A                                                %
%   Output : Physical paramters                                 %
%                                                               %
% -------------------------- Content -------------------------- %

m = 0.9689; L = 0.15; R = 0.125; g = 9.81;                      % Physical parameters
A_R = pi*R^2; rho = 1.225; eta_eff = 0.8;
[J_xx, J_yy, J_zz] = deal(0.01587, 0.01405, 0.02790);           % Moment of Inertia

RPM = 6000; RPS = RPM/60;                                       % Max operating frequency

C_T = (m*g/4) / ( rho*pi*(RPS*2*pi)^2*R^4 );                    % Thrust coefficient (unitless)
C_M = (0.025) / ( rho*pi*(RPS*2*pi)^2*R^5 );                    % Moment coefficient (unitless)

k_T = rho*pi*C_T*R^4;                                           % Thrust constant <==> k_T = 0.5*rho*A_R*C_T*R^2;
k_M = rho*pi*C_M*R^5;                                           % Moment constant <==> k_M = 0.5*rho*A_R*C_M*R^3;

T_max = 4*2*k_T * (RPS*2*pi)^2;                                 % Thrust limit

Mat_Mix = [[k_T,  k_T,   k_T,   k_T];                           % Mixer matrix
           [0, -L*k_T,     0, L*k_T];
           [-L*k_T, 0, L*k_T,     0];
           [-k_M, k_M,  -k_M,  k_M]];

w_hover = sqrt( (m*g)/(4*k_T) );                                % Hovering speed (nominal)