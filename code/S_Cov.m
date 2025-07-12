function [W, V] = S_Cov( dt )
% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : NA                                                 %
%   Output : W & V covariance matrices                          %
%                                                               %
% -------------------------- Content -------------------------- %

I_1 = ones(1,3); freq = 1/dt;

% Process noise parameters
sig_f = 0.002*sqrt(freq);               % Accelerometer noise density [m/s^2/Hz^.5]
sig_w = 0.001*sqrt(freq);               % Angular rate process noise  [rad/s/Hz^.5]
% ------------------------------------- %
W = diag([(1/3)*sig_f^2*dt*I_1, sig_f^2*dt*I_1, sig_w^2*dt*I_1, sig_w^2*dt*I_1]);
% W = [1e-6, 1e-6, 1e-6, 5e-5]          % Good range

% Measurement noise parameters
sig_GPS  = (1/1000)*sqrt(freq);         % GPS SD [m/Hz^.5] (Randowm walk of 1 m per each second)
sig_ZUPT = 0.005;                       % ZUPT precision [m/s]
sig_AHRS = 0.001;                       % AHRS + Magnetometer precision [rad]
sig_gyro = 0.002;                       % Gyroscope precision [rad/s]
% ------------------------------------- %
V = diag([sig_GPS^2*I_1, sig_ZUPT^2*I_1, sig_AHRS^2*I_1, sig_gyro^2*I_1]);
% V = [1e-6, 5e-6, 5e-6, 1e-7]          % Good range