
%% Discretization settings
model.Ts = 0.01;

%% Platooning model
control.ri  = 3;    % [m] Safety Distance
control.h   = 0.5;  % [s] Timegap distance policy
control.L   = 16.5;
control.J = 100;

model.tau   = 0.1;  % [-] Driveline constant

%% Leader settings
leader.t_start         = 1;
leader.t_trans_start   = 5;
leader.t_trans_end     = 15;
leader.Vdes0           = 50/3.6;
leader.Vtrans_amp      = 30/3.6;
leader.Vtrans_f        = 0.1;

%% Controller gains, where: kp&kd >0; kd>kp*tau
control.kp = 0.2;
control.kd = 0.7;
control.kdd = 0;
control.K  = [control.kp, control.kd, control.kdd]; 

%% Determine simulation model
syms qi vi ai ui
x = [qi; vi; ai];
dqi = vi;
dvi = ai;
dai = -1/model.tau * ai + 1/model.tau*ui;
dx = [dqi; dvi; dai];

A_sim = double(jacobian(dx, x));
B_sim = double(jacobian(dx, ui));
clear qi vi ai ui dx dqi dvi dai x