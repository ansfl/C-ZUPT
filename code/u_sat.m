function U_out = u_sat( K_err, L, J_zz, T_max )
% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : NA                                                 %
%   Output : Linearized Dynamics                                %
%                                                               %
% -------------------------- Content -------------------------- %

% * Apply mixer matrix + entire propeller dynamics ... u_cmd --> u_out

[u_1_in, u_2_in, u_3_in, u_4_in] = deal(K_err(1), K_err(2), K_err(3), K_err(4));

% ------------ Actuators boundaries ----------- %
max_thrust = T_max;                                 % Maximum thrust force
[u_1_min, u_1_max] = deal(0.5, max_thrust);     % Min and max thrust forces (3847 < w < 8,600) [RPM]
u_2_max = (u_1_max-u_1_min)*L;                  % Differential thrust x lever arm
u_3_max = u_2_max;                              % Max rolling and pitching torques
u_4_max = 3*(2*pi)*J_zz;                        % Max yawing command (3 revolutions per second)

u_1_out = max(+u_1_min, min(u_1_in, u_1_max));  % Thrust range (positive only)
u_2_out = max(-u_2_max, min(u_2_in, u_2_max));  % Rolling  range (+/-)
u_3_out = max(-u_3_max, min(u_3_in, u_3_max));  % Pitching range (+/-)
u_4_out = max(-u_4_max, min(u_4_in, u_4_max));  % Yawing   range (+/-)

U_out = [u_1_out, u_2_out, u_3_out, u_4_out];

% ------------- Add later ? ------------- %
% u_sat = @(u_x) max(-u_bnd, min(u_bnd, u_x));  % Control law
% w_r = @(x)( exp(-x/5).*sin(x*2) )*0.00;       % Error model (within local linearity !)
% u_k   = u_sat( gain ); %+ w_r(err) ) );       % Apply saturation to control input