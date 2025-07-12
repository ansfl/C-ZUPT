function d_x = fx_Quad(x, u)
% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : NA                                                 %
%   Output : Linearized Dynamics                                %
%                                                               %
% -------------------------- Content -------------------------- %

% clc; clear;
[m, l, g, J_xx, J_yy, J_zz, ~, ~, ~] = sys_params();
J = diag([J_xx, J_yy, J_zz]);

% x = [ones(1,3),zeros(1,9)]; u = [m*g, zeros(1,3)];


X_1_3 = [x(1), x(2), x(3)]'; % NOTE: --,'-- denotes non-conjugate (REAL symbolic variable)
X_4_6 = [x(4), x(5), x(6)]';
X_7_9 = [x(7), x(8), x(9)]';
X_012 = [x(10), x(11), x(12)]';

% Body-to-Inertial Transformation R_B_I
[phi, tht, psi] = deal(x(7), x(8), x(9));      % Keep as is : to enable variables swap

R_B_I = [[cos(tht)*cos(psi), sin(phi)*sin(tht)*cos(psi) - cos(phi)*sin(psi), cos(phi)*sin(tht)*cos(psi) + sin(phi)*sin(psi)];
        [ cos(tht)*sin(psi), sin(phi)*sin(tht)*sin(psi) + cos(phi)*cos(psi), cos(phi)*sin(tht)*sin(psi) - sin(phi)*cos(psi)];
        [         -sin(tht),                              sin(phi)*cos(tht),                              cos(phi)*cos(tht)]];
R_I_B = R_B_I.';

f_skew = @(v) [[0,-v(3),v(2)];[v(3),0,-v(1)];[-v(2),v(1),0]];

[e_x_I, e_y_I, e_z_I] = deal([1,0,0]', [0,1,0]', [0,0,1]');
[e_x_B, e_y_B, e_z_B] = deal(R_I_B*e_x_I, R_I_B*e_y_I, R_I_B*e_z_I);

% Body-rates to inertial-rates Transformation W_eta
W_eta = [[1, sin(phi)*tan(tht), cos(phi)*tan(tht)];
        [0,           cos(phi),         -sin(phi)];
        [0,  sin(phi)/cos(tht), cos(phi)/cos(tht)]];

% --------- Dynamical system function ---------- %
dX_1_3 = R_B_I * X_4_6;
dX_4_6 = (1/m)*u(1)*e_z_I - cross(X_012,X_4_6) - g*e_z_B;
dX_7_9 = W_eta * X_012; 
dX_012 = J^(-1) * ( R_B_I*[u(2), u(3), u(4)]' - f_skew(X_012)*J*X_012 ); % With Coriolis terms

d_x = [dX_1_3; dX_4_6; dX_7_9; dX_012]; 