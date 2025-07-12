
clc; clear all;

%%

dt = 0.01; t_f = 10;
tt = linspace(0, 10, t_f/dt)'; t_len = size(tt,1);


tau = 0.5;
y_k = zeros(t_len,1);
u_k = @(x) 1.5 * (x>2);

t_tau = (dt/tau); 1/t_tau;

for k=1:t_len-1
    y_k(k+1) = y_k(k) + t_tau * ( u_k(tt(k)) - y_k(k) );
end

%

plot(tt, y_k )