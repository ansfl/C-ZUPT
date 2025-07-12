function f_IMU = accelerometer(vel_inp, window_size, dt)
% -------------------------------------------------------------
% STATIONARY  Detects near-stationarity based on accelerometer readings
%
%   is_stationary = stationary(vel, k_zupt, dt, f_stat)
%
%   INPUTS:
%       vel     - velocity vector or matrix [n x N]
%       k_zupt  - window length for backward moving average
%       dt      - sample time [s]
%       f_stat  - specific force threshold [m/s^2]
%
%   OUTPUT:
%       is_stationary - logical scalar (1 if stationary, 0 otherwise)
% -------------------------------------------------------------

vel_smooth = movmean(vel_inp, [window_size 0], 2); % Backward moving average LPF (Causal)
acc_est = diff(vel_smooth, 1, 2) / dt;             % Approximate acceleration via backward finite difference
acc_RMS = sqrt( mean(acc_est.^2, 'all') );         % Compute overall RMS of estimated acceleration
f_IMU = acc_RMS / sqrt(1/dt);                      % Normalized by sampling rate (to converge to instrumental noise)