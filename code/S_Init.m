% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : N/A                                                %
%   Output : Initial conditions                                 %
%                                                               %
% -------------------------- Content -------------------------- %

% Initialization of control vectors
u_out_k = zeros(u_dim, length(tt)); 
u_cmd_k = zeros(u_dim, length(tt));

% C-ZUPT parameters (choose false to de-activate)
flag_zupt = false;                               % Boolean
idx_zupt = zeros(1, length(tt));                 % Time index : detection periods
V_zupt   = zeros(1, length(tt));                 % Time index : velocity estimates
f_IMU    = zeros(1, length(tt));                 % Acceleration array
k_zupt = 150;                                    % Number of past samples
f_stat = 0.05;                                   % Maximum specific force for stationarity detection
v_stat = 0.4;

% Initialization of state vectors
x_std = zeros(x_dim, length(tt)); y_k = x_std;
x_k = x_std; x_k(:,1) = x_0;
x_e = x_std; x_e(:,1) = x_0;

x_idx = (1:x_dim)';                             % Full obsverability
v_idx = [4,5,6]';
w_k = @() randn( size(A*x_k(:,1)) ).*sqrt(diag(W));
v_k = @() randn( size(x_idx) ).*sqrt(diag(V));

% Solution loop
A_k = expm(A*dt); B_k = B*dt;                % Discrete-time solution
P_k = 0.1*eye(x_dim);                        % Initialize error covariance