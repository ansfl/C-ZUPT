clc; clear; close all;

set(0, 'DefaultFigureRenderer', 'painters')
set(0, 'defaultfigurecolor', [1 1 1]);
set(0, 'DefaultAxesXGrid', 'on');
set(groot, 'DefaultLineLineWidth', 2);
set(0, 'DefaultTextInterpreter', 'latex');   % For all text objects (titles, labels, etc.)
set(0, 'DefaultAxesTickLabelInterpreter', 'latex');  % For axis tick labels
set(0, 'DefaultLegendInterpreter', 'latex'); % For legends

fig_loc = [2500 680 750 700];
Fig = @(fig_loc) figure('rend', 'painters', 'pos', fig_loc);

% ----------- Initial Condition ------------ % (Hovering setpoint)
O_3 = [0,0,0]; I_1 = [1,1,1];                % Utils
X_ref = [O_3, O_3, O_3, O_3]';               % Desired setpoint
x_0   = [-I_1, O_3, O_3+0.1, O_3]';          % Initial point

% ------------- Configuration -------------- %
rho = 1;                                     % Update frequency (INTEGER !)
dt = 0.001; t_f = 10; tt = 0:dt:t_f;         % Time domain
time_const = 10*dt; t_tau = dt/time_const;   % Control response
Rad_2_deg = 360/(2*pi); Rad_2_RPM = 60/(2*pi);

% Linearized system dynamics
[A, B] = Linearize_Quad( ); 
x_dim = size(A, 1);                          % Number of states
u_dim = size(B, 2);                          % Number of inputs

% Get parameters
[m, L, g, J_xx, J_yy, J_zz, k_T, k_M, eta_eff, Mat_Mix, w_hover, T_max] = sys_params();

% Covariance matrices (process & measurement)
[W, V] = S_Cov( dt );

% Control Weights
[Q_lqr, R_lqr] = S_Bryson( );                % Bryson's rule
K = lqr(A, B, Q_lqr, R_lqr);

S_Init;                                      % Initial conditions
S_Solve;                                     % Solution loop

X_est = x_e;                                 % Estimate vector
X_GT  = x_k;                                 % Ground-true vector
x_std(:,1) = x_std(:,2);                     % Compensate for k+1 index (avoid ZERO)

% Observability rank (Uncomment)
% rank(obsv(A, h_x(obs_entries, x_dim) ));

%% -------------------------------------------- %
% --------- Analysis & Visualization ---------- %
% --------------------------------------------- %

l_w = 2.5; f_col = [0.8 0.4 0.1];
Fig([3000 700 550 450]);

% -------------- Position -------------- %
subplot(4, 3, 1); hold on; 
plot(tt, X_GT(1,:)); plot(tt, X_est(1,:), color=f_col, LineWidth=l_w, LineStyle='--');
ylabel('$\xi_x^I(t)$', fontsize=14);

subplot(4, 3, 2); hold on;
plot(tt, X_GT(2,:)); plot(tt, X_est(2,:), color=f_col, LineWidth=l_w, LineStyle='--');
ylabel('$\xi_y^I(t)$', fontsize=14);

subplot(4, 3, 3); hold on;
plot(tt, X_GT(3,:)); plot(tt, X_est(3,:), color=f_col, LineWidth=l_w, LineStyle='--');
ylabel('$\xi_z^I(t)$', fontsize=14);

y_all = X_GT(1:3, :); ylim_shared = [min(y_all(:)), max(y_all(:))];
arrayfun(@(i) set(subplot(4,3,i), 'YLim', ylim_shared), 1:3);

% -------------- Velocity -------------- %
subplot(4, 3, 4); hold on; 
plot(tt, X_GT(4,:)); plot(tt, X_est(4,:), color=f_col, LineWidth=l_w, LineStyle='--');
ylabel('$v_x^B(t)$', fontsize=14);

subplot(4, 3, 5); hold on;
plot(tt, X_GT(5,:)); plot(tt, X_est(5,:), color=f_col, LineWidth=l_w, LineStyle='--');
ylabel('$v_y^B(t)$', fontsize=14);

subplot(4, 3, 6); hold on;
plot(tt, X_GT(6,:)); plot(tt, X_est(6,:), color=f_col, LineWidth=l_w, LineStyle='--');
ylabel('$v_z^B(t)$', fontsize=14);

y_all = X_GT(4:6, :); ylim_shared = [min(y_all(:)), max(y_all(:))];
arrayfun(@(i) set(subplot(4,3,i), 'YLim', ylim_shared), 4:6);

% -------------- Orientation -------------- %
wrap_to_pi = @(angle) mod(angle + pi, 2*pi) - pi;

subplot(4, 3, 7); hold on; 
plot(tt, wrap_to_pi(X_GT(7,:))*Rad_2_deg); % plot(tt, X_est(7,:)*Rad_2_deg, color=f_col, LineWidth=l_w, LineStyle='-.');
plot(tt, wrap_to_pi(X_est(7,:))*Rad_2_deg, color=f_col, LineWidth=l_w, LineStyle='--');
ylabel('$\phi(t)$', fontsize=14);

subplot(4, 3, 8); hold on;
plot(tt, wrap_to_pi(X_GT(8,:))*Rad_2_deg); % plot(tt, X_est(8,:)*Rad_2_deg, color=f_col, LineWidth=l_w, LineStyle='-.');
plot(tt, wrap_to_pi(X_est(8,:))*Rad_2_deg, color=f_col, LineWidth=l_w, LineStyle='--');
ylabel('$\theta(t)$', fontsize=14);

subplot(4, 3, 9); hold on;
plot(tt, wrap_to_pi(X_GT(9,:))*Rad_2_deg); % plot(tt, X_est(9,:)*Rad_2_deg, color=f_col, LineWidth=l_w, LineStyle='-.');
plot(tt, wrap_to_pi(X_est(9,:))*Rad_2_deg, color=f_col, LineWidth=l_w, LineStyle='--');
ylabel('$\psi(t)$', fontsize=14);

y_all = X_GT(7:9, :); ylim_shared = [min(y_all(:))*Rad_2_deg, max(y_all(:))*Rad_2_deg];
arrayfun(@(i) set(subplot(4,3,i), 'YLim', ylim_shared), 7:9);

% -------------- Angular rates -------------- %
subplot(4, 3, 10); hold on; 
plot(tt, X_GT(10,:)*Rad_2_deg); plot(tt, X_est(10,:)*Rad_2_deg, color=f_col, LineWidth=l_w, LineStyle='--');
ylabel('$\omega_x^B(t)$', fontsize=14);
xlabel('Time [s]', fontsize=14);

subplot(4, 3, 11); hold on;
plot(tt, X_GT(11,:)*Rad_2_deg); plot(tt, X_est(11,:)*Rad_2_deg, color=f_col, LineWidth=l_w, LineStyle='--');
ylabel('$\omega_y^B(t)$', fontsize=14);
xlabel('Time [s]', fontsize=14);

subplot(4, 3, 12); hold on;
plot(tt, X_GT(12,:)*Rad_2_deg); plot(tt, X_est(12,:)*Rad_2_deg, color=f_col, LineWidth=l_w, LineStyle='--');
ylabel('$\omega_z^B(t)$', fontsize=14);
xlabel('Time [s]', fontsize=14);

y_all = X_GT(10:12, :); ylim_shared = [min(y_all(:))*Rad_2_deg, max(y_all(:))*Rad_2_deg];
arrayfun(@(i) set(subplot(4,3,i), 'YLim', ylim_shared), 10:12);
arrayfun(@(i) yline(subplot(4,3,i), 0, color='k', LineWidth=l_w/2, LineStyle='-.', alpha=0.8), 1:12);
arrayfun(@(i) grid(subplot(4, 3, i), 'on'), 1:12);
% LEGEND: x_GT, x_est, x_Ref !

% legend({'$\boldmath{x}_{GT}$', '$\hat{\boldmath{x}}$'}, 'Interpreter', 'latex', 'Orientation', 'horizontal', 'Location', 'northoutside', 'FontSize', 12);
% exportgraphics(gcf, 'Fig_States_1.png', 'Resolution', 300); % 300 DPI for high resolution

%% Control vs. Time
% f_col = [0.8 0.4 0.1];
Fig([2400 600 550 450]);
[x_l, x_r] = deal(0.0, 3.0);

% % -------------- Position -------------- %
subplot(4, 1, 1); hold on; 
plot(tt, u_cmd_k(1,:)); plot(tt, u_out_k(1,:), color=f_col, LineWidth=l_w, LineStyle='--');
yline(m*g, color='k', LineWidth=l_w/2, LineStyle='-.');
ylabel('$u_z(t)$', fontsize=14); xlim([x_l, x_r]);

subplot(4, 1, 2); hold on;
plot(tt, u_cmd_k(2,:)); plot(tt, u_out_k(2,:), color=f_col, LineWidth=l_w, LineStyle='--');
yline(0, color='k', LineWidth=l_w/2, LineStyle='-.');
ylabel('$u_{\phi}(t)$', fontsize=14); xlim([x_l, x_r]);

subplot(4, 1, 3); hold on;
plot(tt, u_cmd_k(3,:)); plot(tt, u_out_k(3,:), color=f_col, LineWidth=l_w, LineStyle='--');
yline(0, color='k', LineWidth=l_w/2, LineStyle='-.');
ylabel('$u_{\theta}(t)$', fontsize=14); xlim([x_l, x_r]);

subplot(4, 1, 4); hold on; 
plot(tt, u_cmd_k(4,:)); plot(tt, u_out_k(4,:), color=f_col, LineWidth=l_w, LineStyle='--');
yline(0, color='k', LineWidth=l_w/2, LineStyle='-.');
ylabel('$u_{\psi}(t)$', fontsize=14);
xlabel('Time [s]', fontsize=14); xlim([x_l, x_r]);
% % exportgraphics(gcf, 'Fig_Control_2.png', 'Resolution', 300); % 300 DPI for high resolution


%% Rotor speeds vs. Time

Omega_squared = zeros(4,k);       % preallocate
for i = 1:k
    Omega_squared(:, i) = lsqnonneg(Mat_Mix, u_out_k(:,i));
end

w_omg = Omega_squared.^.5;

f_col = [0.8 0.4 0.1];
Fig([2400 600 500 400]);
[y_l, y_r] = deal(0.4*w_hover*Rad_2_RPM, 1.7*w_hover*Rad_2_RPM);

% % -------------- Position -------------- %
subplot(2, 2, 1); hold on;
plot(tt, w_omg(1,:)*Rad_2_RPM, LineWidth=l_w/2); ylabel('$-\Omega_1(t)$', fontsize=14); ylim([y_l, y_r]);
yline(w_hover*Rad_2_RPM, color='k', LineWidth=l_w/2, LineStyle='-.');

subplot(2, 2, 2); hold on;
plot(tt, w_omg(2,:)*Rad_2_RPM, LineWidth=l_w/2); ylabel('$+\Omega_2(t)$', fontsize=14); ylim([y_l, y_r]);
yline(w_hover*Rad_2_RPM, color='k', LineWidth=l_w/2, LineStyle='-.');

subplot(2, 2, 4); hold on;
plot(tt, w_omg(3,:)*Rad_2_RPM, LineWidth=l_w/2); ylabel('$-\Omega_3(t)$', fontsize=14); ylim([y_l, y_r]);
yline(w_hover*Rad_2_RPM, color='k', LineWidth=l_w/2, LineStyle='-.');
xlabel('Time [s]', fontsize=14); % xlim([x_l, x_r]);

subplot(2, 2, 3); hold on;
plot(tt, w_omg(4,:)*Rad_2_RPM, LineWidth=l_w/2); ylabel('$+\Omega_4(t)$', fontsize=14); ylim([y_l, y_r]);
yline(w_hover*Rad_2_RPM, color='k', LineWidth=l_w/2, LineStyle='-.');
xlabel('Time [s]', fontsize=14); % xlim([x_l, x_r]);
% exportgraphics(gcf, 'Fig_RPM_2.png', 'Resolution', 300); % 300 DPI for high resolution

%% ------------- C-ZUPT Analysis -------------- %

a_col = [0.2 0.35 1]; b_col = [.88 .3 .1]; c_col = [.3 .8 0.1]; % c_col = [0.9 0.9 0.0];

Fig([3000 700 550 250]); l_w = 1.25;

yyaxis left;
plot(tt, f_IMU, color=a_col, LineWidth=l_w); ylabel('$\| \tilde{f} (t) \|_2$', fontsize=14);
yline(f_stat, color=a_col, LineWidth=l_w*1.5, LineStyle='-.'); 

yyaxis right;
plot(tt, V_zupt, LineWidth=l_w*1., color=b_col, LineStyle='-'); 
yline(v_stat, color=b_col, LineWidth=l_w*1.5, LineStyle='-.');
ylabel('$\| \hat{v}^B (t) \|_2$', fontsize=14);

highlight_segments(tt, idx_zupt, c_col, 0.4);
xlabel('Time [s]', 'FontSize', 14);
xlim([0 7]);

% exportgraphics(gcf, 'Fig_ZUPT_4.png', 'Resolution', 300); % 300 DPI for high resolution

%% Normalized Estimation Uncertainty

zeta_1 = sum(x_std,1) ./ sum(x_std(:,1));

Fig([3000 700 550 350]); l_w = .5;
semilogy(tt, zeta_1); ylabel('$ \tilde{\zeta} (t) $', fontsize=14, LineWidth=l_w);
highlight_segments(tt, idx_zupt, c_col, 0.4);
xlabel('Time [s]', 'FontSize', 14);
xlim([0 7]);

% exportgraphics(gcf, 'Fig_ZUPT_5.png', 'Resolution', 300); % 300 DPI for high resolution

%% Battery model analysis

% Extract power function
P_in  = (k_M/eta_eff) * w_omg.^3;
P_out = sum(P_in, 1);
[I_t, V_t, V_oc, SoC] = Battery_dynamics( tt, P_out ); 

l_w = 1.5; c_col = [.3 .8 0.1];                            % Pale green
alpha = 0.0015;                                            % Adjust (0 < alpha < 1)
P_smooth = filter(alpha, [1 alpha-1], P_out);              % Filtered version     
I_smooth = filter(alpha, [1 alpha-1], I_t);                % Filtered version

Fig([2200 680 600 500]);
subplot(2,2,1); plot(tt, P_smooth); ylabel('$P(t)$ [W]', fontsize=14); grid on;
subplot(2,2,3); plot(tt, I_smooth); ylabel('$I(t)$ [A]', fontsize=14); grid on; xlabel('Time [s]', fontsize=14); grid on;
subplot(2,2,2); plot(tt, V_oc); ylabel('$V_{oc}(t)$ [V]', fontsize=14); grid on; % xlabel('Time [s]', fontsize=14);
subplot(2,2,4); plot(tt, SoC); ylabel('SoC(t)', fontsize=14); xlabel('Time [s]', fontsize=14); grid on;
% exportgraphics(gcf, 'Fig_Battery_2.png', 'Resolution', 300); % 300 DPI for high resolution


%% Error Analysis (Estimation, Control, Effort)

l_w = 2.3;
err_est = [vecnorm(X_GT(1:3,:) - X_est(1:3,:)); vecnorm(X_GT(4:6,:) - X_est(4:6,:)); vecnorm(X_GT(7:9,:) - X_est(7:9,:)); vecnorm(X_GT(10:12,:) - X_est(10:12,:))];
err_ctr = [vecnorm(X_GT(1:3,:) - X_ref(1:3,:)); vecnorm(X_GT(4:6,:) - X_ref(4:6,:)); vecnorm(X_GT(7:9,:) - X_ref(7:9,:)); vecnorm(X_GT(10:12,:) - X_ref(10:12,:))];


Fig([3000 700 550 450]);

% -------------- Position -------------- %
subplot(4, 3, 1); hold on; 
plot(tt, err_est(1,:), LineWidth=l_w/2);
ylabel('$\| {e}_{\xi^I} (t) \|$', fontsize=14);

subplot(4, 3, 2); hold on;
plot(tt, err_ctr(1,:), LineWidth=l_w/2);
ylabel('$\| {\varepsilon}_{\xi^I} (t) \|$', fontsize=14);

subplot(4, 3, 3); hold on;
plot(tt, u_out_k(1,:), LineWidth=l_w/2); ylabel('$ u_z (t)$', fontsize=14);

% -------------- Velocity -------------- %
subplot(4, 3, 4); hold on; 
plot(tt, err_est(2,:), LineWidth=l_w/2); ylabel('$\| {e}_{v^I} (t) \|$', fontsize=14);

subplot(4, 3, 5); hold on;
plot(tt, err_ctr(2,:), LineWidth=l_w/2); ylabel('$\| {\varepsilon}_{v^B} (t) \|$', fontsize=14);

subplot(4, 3, 6); hold on;
plot(tt, u_out_k(2,:), LineWidth=l_w/2); ylabel('$ u_\phi (t) $', fontsize=14);

% -------------- Orientation -------------- %
wrap_to_pi = @(angle) mod(angle + pi, 2*pi) - pi;

subplot(4, 3, 7); hold on; 
plot(tt, (err_est(3,:))*Rad_2_deg, LineWidth=l_w/2); ylabel('$\| {e}_{\eta^I} (t) \|$', fontsize=14);

subplot(4, 3, 8); hold on;
plot(tt, (err_ctr(3,:))*Rad_2_deg, LineWidth=l_w/2); ylabel('$\| {\varepsilon}_{\eta^I} (t) \|$', fontsize=14);

subplot(4, 3, 9); hold on;
plot(tt, u_out_k(3,:), LineWidth=l_w/2); ylabel('$ {u}_{\theta} (t) $', fontsize=14);

% -------------- Angular rates -------------- %
subplot(4, 3, 10); hold on; 
plot(tt, err_est(4,:)*Rad_2_deg, LineWidth=l_w/2); ylabel('$\| {e}_{\omega^B} (t) \|$', fontsize=14);
xlabel('Time [s]', fontsize=14);

subplot(4, 3, 11); hold on;
plot(tt, err_ctr(4,:)*Rad_2_deg, LineWidth=l_w/2); ylabel('$\| {\varepsilon}_{\omega^B} (t) \|$', fontsize=14);
xlabel('Time [s]', fontsize=14);

subplot(4, 3, 12); hold on;
plot(tt, u_out_k(4,:), LineWidth=l_w/2); ylabel('${u}_{\psi} (t) $', fontsize=14);
xlabel('Time [s]', fontsize=14);

% % legend({'$\boldmath{x}_{GT}$', '$\hat{\boldmath{x}}$'}, 'Interpreter', 'latex', 'Orientation', 'horizontal', 'Location', 'northoutside', 'FontSize', 12);
% exportgraphics(gcf, 'Fig_ZUPT_err.png', 'Resolution', 300);