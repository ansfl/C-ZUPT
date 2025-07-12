function [A_s, B_s] = Linearize_Quad( )
% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : NA                                                 %
%   Output : Linearized Dynamics                                %
%                                                               %
% -------------------------- Content -------------------------- %


% ---------------------- State variables ---------------------- %
syms x_1 x_2 x_3                           % Position coordinates   (inertial)
syms x_4 x_5 x_6                           % Velocity coordinates   (body)
syms x_7 x_8 x_9                           % Euler angles           (inertial)
syms x_10 x_11 x_12                        % Angluar velocity       (body)

% --------------------- Control variables --------------------- %
syms u_1 u_2 u_3 u_4                                            % Control variables (roll, pitch, yaw, thrust)
syms m L g J_x J_y J_z                                       % System parameters

X_1_3 = [x_1, x_2, x_3].'; % NOTE: --,'-- denotes non-conjugate (REAL symbolic variable)
X_4_6 = [x_4, x_5, x_6].';
X_7_9 = [x_7, x_8, x_9].';
X_012 = [x_10, x_11, x_12].';
J = diag([J_x, J_y, J_z]);

% Body-to-Inertial Transformation R_B_I
[phi, tht, psi] = deal(x_7, x_8, x_9);      % Keep as is : to enable variables swap

% Full nonlinear version
% R_B_I = [[cos(tht)*cos(psi), sin(phi)*sin(tht)*cos(psi) - cos(phi)*sin(psi), cos(phi)*sin(tht)*cos(psi) + sin(phi)*sin(psi)];
        % [ cos(tht)*sin(psi), sin(phi)*sin(tht)*sin(psi) + cos(phi)*cos(psi), cos(phi)*sin(tht)*sin(psi) - sin(phi)*cos(psi)];
        % [         -sin(tht),                              sin(phi)*cos(tht),                              cos(phi)*cos(tht)]];

f_skew = @(v) [[0,-v(3),v(2)];[v(3),0,-v(1)];[-v(2),v(1),0]];
X_eta = [phi, tht, psi];                            % Keep as is : to enable variables swap
R_B_I = eye(3) + f_skew(X_eta);                     % Small angles approximation
R_I_B = R_B_I.';

[e_x_I, e_y_I, e_z_I] = deal([1,0,0]', [0,1,0]', [0,0,1]');
[e_x_B, e_y_B, e_z_B] = deal(R_I_B*e_x_I, R_I_B*e_y_I, R_I_B*e_z_I);

% body-rates to inertial-rates Transformation W_eta
% W_eta = [[1, sin(phi)*tan(tht), cos(phi)*tan(tht)];
        % [0,           cos(phi),         -sin(phi)];
        % [0,  sin(phi)/cos(tht), cos(phi)/cos(tht)]];

W_eta = eye(3); % Small angles approximation (For linearization!)

% --------- Dynamical system function ---------- %
dX_1_3 = R_B_I * X_4_6;
dX_4_6 = (1/m)*u_1*e_z_I - cross(X_012,X_4_6) - g*e_z_B;
dX_7_9 = W_eta * X_012;
dX_012 = J^(-1) * ( R_B_I*[u_2, u_3, u_4].' - f_skew(X_012)*J*X_012 ); % With Coriolis terms

f_x = [dX_1_3; dX_4_6; dX_7_9; dX_012]; 
x_t = [X_1_3; X_4_6; X_7_9; X_012];
u_t = [u_1, u_2, u_3, u_4]; % NOTE: All inputs are already IN body frame !

% Compute Jacobians (symbolic)
J_A = jacobian( f_x, x_t);
J_B = jacobian( f_x, u_t);

% Compute Jacobians about --Equilibrium Point (P)--
syms p_x p_y p_z;
O_3 = [0,0,0]; x_e_3 = [p_x, p_y, p_z];
x_eq = [x_e_3, O_3, O_3, O_3]';                  % Equilibrium point: States
u_eq = [m*g, 0, 0, 0];                          % Equilibrium point: Inputs

A_sym = subs(J_A, u_t, u_eq);
A_sym = subs(A_sym, x_t, x_eq);

B_sym = subs(J_B, u_t, u_eq);
B_sym = subs(B_sym, x_t, x_eq);

% Compute Jacobians (numerical values)
[m_s, ~, g_s, J_xx, J_yy, J_zz, ~, ~, ~, ~] = sys_params();
A_s = double(subs(A_sym, g, g_s));
B_s = double(subs(B_sym, [m J_x J_y J_z], [m_s J_xx, J_yy, J_zz]));


% --------------------- Code Junkyard --------------------- %
% dX_012 = J^(-1)* ( (u_1*e_x + u_2*e_y + u_3*e_z) ); % Neglecting Coriolis (For linearization !)
% dX_012 = J^(-1) * ( eye(3)*[u_2, u_3, u_4].' - cross(X_012, J*X_012 ) ); % With Coriolis terms

% f_x = [dx_1; dx_2; dx_3; dx_4; dx_5; dx_6; dx_7; dx_8; dx_9; dx_10; dx_11; dx_12];
% x_t = [x_1, x_2, x_3, x_4, x_5, x_6, x_7, x_8, x_9, x_10, x_11, x_12];