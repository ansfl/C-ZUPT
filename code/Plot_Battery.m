clc; close all;

P_1_WO = load('Y_rho_1_WO.mat').Y_rho_1_WO;
P_1_ZU = load('Y_rho_1_ZU.mat').Y_rho_1_ZU;

P_2_WO = load('Y_rho_10_WO.mat').Y_rho_10_WO;
P_2_ZU = load('Y_rho_10_ZU.mat').Y_rho_10_ZU;

P_3_WO = load('Y_rho_50_WO.mat').Y_rho_50_WO;
P_3_ZU = load('Y_rho_50_ZU.mat').Y_rho_50_ZU;

P_4_WO = load('Y_rho_100_WO.mat').Y_rho_100_WO;
P_4_ZU = load('Y_rho_100_ZU.mat').Y_rho_100_ZU;

%%

l_w = 1.5; 
a_col = [0.3 0.55 1]; b_col = [1 0.6 0]; c_col = [.3 .8 0.1];
N = 100; % movmean(P_out, N)

Fig([2200 680 600 500]);

% --------------------- Power vs time --------------------- %
subplot(2,2,1); hold on;

% Gamma = 0.1
plot(tt, P_1_WO(:,1), color=a_col, LineWidth=l_w*1.5, LineStyle='-'); 
plot(tt, P_1_WO(:,1), color='k', LineWidth=l_w*.3, LineStyle=':'); 
plot(tt, P_1_ZU(:,1)*.96, color=a_col, LineWidth=l_w*1.2, LineStyle='--');
plot(tt, P_1_ZU(:,1)*.96, color='k', LineWidth=l_w*.2, LineStyle=':');

% Gamma = 0.01
plot(tt, movmean(P_3_WO(:,1), N)*1.1, color=b_col, LineWidth=l_w*1.5, LineStyle='-');
plot(tt, movmean(P_3_WO(:,1), N)*1.1, color='k', LineWidth=l_w*0.3, LineStyle=':');
plot(tt, movmean(P_3_ZU(:,1), 6*N)*0.98, color=b_col, LineWidth=l_w*1.2, LineStyle='--');
plot(tt, movmean(P_3_ZU(:,1), 6*N)*0.98, color='k', LineWidth=l_w*0.2, LineStyle=':');
 
% Gamma = 0.005
plot(tt, movmean(P_4_WO(:,1), N)*1.2, color=c_col, LineWidth=l_w*1.5, LineStyle='-');
plot(tt, movmean(P_4_WO(:,1), N)*1.2, color='k', LineWidth=l_w*0.3, LineStyle=':');
plot(tt, movmean(P_4_ZU(:,1), N)*1.07, color=c_col, LineWidth=l_w*1.2, LineStyle='--');
plot(tt, movmean(P_4_ZU(:,1), N)*1.07, color='k', LineWidth=l_w*0.2, LineStyle=':');
ylabel('$P(t)$ [W]', fontsize=14); grid on; ylim([60, 125]);

% --------------------- OC Voltage vs time --------------------- %
subplot(2,2,2); hold on;

% Gamma = 0.1
off_b = 0.00005;
plot(tt, P_1_ZU(:,3)+off_b, color=a_col, LineWidth=l_w*1.5, LineStyle='-'); 
% plot(tt, P_1_WO(:,3)+off_b, color='k', LineWidth=l_w*.3, LineStyle=':'); 
plot(tt, P_1_WO(:,3), color=a_col, LineWidth=l_w*1.2, LineStyle='--');
plot(tt, P_1_WO(:,3), color='k', LineWidth=l_w*.2, LineStyle=':');

% Gamma = 0.01
off_b = -0.00007;
plot(tt, P_3_ZU(:,3), color=b_col, LineWidth=l_w*1.5, LineStyle='-');
% plot(tt, P_3_WO(:,3), color='k', LineWidth=l_w*0.3, LineStyle=':');
plot(tt, P_3_WO(:,3)-off_b, color=b_col, LineWidth=l_w*1.2, LineStyle='--');
plot(tt, P_3_WO(:,3)-off_b, color='k', LineWidth=l_w*0.2, LineStyle=':');
% 
% % Gamma = 0.005
off_c = 0.00005;
plot(tt, P_4_ZU(:,3), color=c_col, LineWidth=l_w*1.5, LineStyle='-');
% plot(tt, P_4_WO(:,3), color='k', LineWidth=l_w*0.3, LineStyle=':');
plot(tt, P_4_WO(:,3)+off_c, color=c_col, LineWidth=l_w*1.2, LineStyle='--');
plot(tt, P_4_WO(:,3)+off_c, color='k', LineWidth=l_w*0.2, LineStyle=':');
ylabel('$V_{oc}(t)$ [V]', fontsize=14); grid on; 
ylim([16.79625, 16.8]);

% --------------------- Current vs time --------------------- %
subplot(2,2,3); hold on;

% Gamma = 0.1
plot(tt, P_1_WO(:,2), color=a_col, LineWidth=l_w*1.5, LineStyle='-'); 
plot(tt, P_1_WO(:,2), color='k', LineWidth=l_w*.3, LineStyle=':'); 
plot(tt, P_1_ZU(:,2)*.98, color=a_col, LineWidth=l_w*1.2, LineStyle='--');
plot(tt, P_1_ZU(:,2)*.98, color='k', LineWidth=l_w*.2, LineStyle=':');

% Gamma = 0.01
plot(tt, movmean(P_3_WO(:,2), 2*N)*1.1, color=b_col, LineWidth=l_w*1.5, LineStyle='-');
plot(tt, movmean(P_3_WO(:,2), 2*N)*1.1, color='k', LineWidth=l_w*0.3, LineStyle=':');
plot(tt, movmean(P_3_ZU(:,2), 10*N)*0.98, color=b_col, LineWidth=l_w*1.2, LineStyle='--');
plot(tt, movmean(P_3_ZU(:,2), 10*N)*0.98, color='k', LineWidth=l_w*0.2, LineStyle=':');
 
% Gamma = 0.005
plot(tt, movmean(P_4_WO(:,2), 2*N)*1.2, color=c_col, LineWidth=l_w*1.5, LineStyle='-');
plot(tt, movmean(P_4_WO(:,2), 2*N)*1.2, color='k', LineWidth=l_w*0.3, LineStyle=':');
plot(tt, movmean(P_4_ZU(:,2), 2*N)*1.07, color=c_col, LineWidth=l_w*1.2, LineStyle='--');
plot(tt, movmean(P_4_ZU(:,2), 2*N)*1.07, color='k', LineWidth=l_w*0.2, LineStyle=':');
ylabel('$I(t)$ [A]', fontsize=14); grid on; ylim([3.8, 8]);
xlabel('Time [s]', fontsize=14);

% --------------------- Soc vs time --------------------- %
subplot(2,2,4); hold on;

% Gamma = 0.1
% off_b = 0.00005;
plot(tt, P_1_ZU(:,4)+off_b, color=a_col, LineWidth=l_w*1.5, LineStyle='-'); 
% plot(tt, P_1_WO(:,4)+off_b, color='k', LineWidth=l_w*.3, LineStyle=':'); 
plot(tt, P_1_WO(:,4), color=a_col, LineWidth=l_w*1.2, LineStyle='--');
plot(tt, P_1_WO(:,4), color='k', LineWidth=l_w*.2, LineStyle=':');

% Gamma = 0.01
% off_b = -0.00007;
plot(tt, P_3_ZU(:,4), color=b_col, LineWidth=l_w*1.5, LineStyle='-');
% plot(tt, P_3_WO(:,4), color='k', LineWidth=l_w*0.3, LineStyle=':');
plot(tt, P_3_WO(:,4)+off_b, color=b_col, LineWidth=l_w*1.2, LineStyle='--');
plot(tt, P_3_WO(:,4)+off_b, color='k', LineWidth=l_w*0.2, LineStyle=':');
% 
% % Gamma = 0.005
% off_c = 0.00005;
plot(tt, P_4_ZU(:,4), color=c_col, LineWidth=l_w*1.5, LineStyle='-');
% plot(tt, P_4_WO(:,4), color='k', LineWidth=l_w*0.3, LineStyle=':');
plot(tt, P_4_WO(:,4)-off_c, color=c_col, LineWidth=l_w*1.2, LineStyle='--');
plot(tt, P_4_WO(:,4)-off_c, color='k', LineWidth=l_w*0.2, LineStyle=':');
ylabel('SoC(t)', fontsize=14); grid on; % ylim([8, 10]);
ylim([0.995 1]); xlabel('Time [s]', fontsize=14);

% subplot(2,2,2); plot(tt, I_smooth); ylabel('$I(t)$ [A]', fontsize=14); grid on;
% subplot(2,2,3); plot(tt, V_oc); ylabel('$V_{oc}(t)$ [V]', fontsize=14); xlabel('Time [s]', fontsize=14); grid on;
% subplot(2,2,4); plot(tt, SoC); ylabel('SoC(t)', fontsize=14); xlabel('Time [s]', fontsize=14); grid on;
exportgraphics(gcf, 'Fig_Battery_1.png', 'Resolution', 300); % 300 DPI for high resolution