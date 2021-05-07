% Reid Smith, Spring 2021
% Code seeks to take in inputs Z and use design variables to optimize
% performance of coupling variables

clear
clc
close all


%% Run Simulation

% KDE2814XF-515
Y_bar_q.K_tau = 0.0185;
Y_bar_q.R_phase = 0.013;
Y_bar_q.active_mass = 0.125;

% Initialize Z_in
Z.I_rms = 24;
Z.V = 26.1;
Z.wr = 13e3*2*pi/60;
v_m = [1;1;1];
w_m = [0;0;0];

[Y_out] = ATC_lower_fcn(Z,Y_bar_q,v_m,w_m)


%% Lower ATC Fcn

function [Y_bar_m] = ATC_lower_fcn(Z,Y_bar_q,v_m,w_m)
% u is structure of inputs
u.T_begin = 20;    % starting temperature (landing temperature)
u.T_amb = 20;       % ambient temperature
u.T_margin = u.T_begin + 115;   % winding threshold temperature
u.vel_air = 22;         % cooling air velocity
u.I = Z.I_rms;
u.V = Z.V;
u.wr = Z.wr;

input_cell = cell(1,4);
input_cell{1} = Y_bar_q;
input_cell{2} = u;
input_cell{3} = v_m;
input_cell{4} = w_m;


x0 = [0.1,0.1,12];
LB = [0,0,0];
options = optimset('disp','iter','TolFun',1e-3,'TolX',1e-9,'Algorithm','Interior-Point');
[xval,fval] = fmincon(@(x)obj_low_ATC(x,input_cell),x0,[],[],[],[],LB,[],@(x)constr_low_ATC(x,input_cell),...
    options);
u.DF = xval(1);
u.SLF = xval(2);
u.poles = xval(3);

% Use final temp to determine function values
tspan = [0,500];
y0 = ones(13,1)*u.T_begin;
[~,y] = ode23tb(@(t,y) motorThermal(t,y,u), tspan, y0);
max_ss = max(y(:,end));
y0 = ones(13,1)*max_ss;
[~,~,Y_bar_m,G]  = thermal_model_Joby_0427_mod(u,y0);

% Display Results
fprintf("\nDiameter: %3e\nLength: %3e\nPoles: %3e\n",G.Ro,G.L,u.poles)

end

%% Functions

function [f_obj] = obj_low_ATC(x,input_cell)
Y_in = input_cell{1};
u = input_cell{2};
v_m = input_cell{3};
w_m = input_cell{4};
u.DF = x(1);
u.SLF = x(2);
u.poles = x(3);

% Use final temp to determine function values
tspan = [0,500];
y0 = ones(13,1)*u.T_begin;
[~,y] = ode23tb(@(t,y) motorThermal(t,y,u), tspan, y0);
max_ss = max(y(:,end));
y0 = ones(13,1)*max_ss;

% Run model to determine estimates
[~,~,Y_out,~]  = thermal_model_Joby_0427_mod(u,y0);
% Generate c_2 and do preliminary scaling
c_m = [(Y_in.K_tau - Y_out.K_tau)/Y_in.K_tau; ...
    (Y_in.R_phase - Y_out.R_phase)/Y_in.R_phase;...
    (Y_in.active_mass - Y_out.active_mass)/Y_in.active_mass];
% f_obj = norm(v_m.*c_m + (w_m.*c_m).^2);
f_obj = norm(c_m);
end

% Constraint function
function [c,ceq] = constr_low_ATC(x,input_cell)
% Y_in = input_cell{1};
u = input_cell{2};
u.DF = x(1);
u.SLF = x(2);
u.poles = x(3);

tspan = [0,500];
y0 = ones(13,1)*u.T_begin;

% Check for geometry violations
[~,~,~,G]  = thermal_model_Joby_0427_mod(u,y0);
c(1) = G.d_wire - G.Dw;
c(2) = G.d_wire - G.Ww;
c(3) = 2 - G.n_turns;
c(4) = 2 - G.n_pp;
c(5) = 2 - x(3);

% Check for temperature violations

if any(sign(c(1:5)) > 0)        % Constraint not met
    c(6) = 1;       % Show constraint not met
else
    [~,y] = ode23tb(@(t,y) motorThermal(t,y,u), tspan, y0);
    max_ss = max(y(:,end));
    c(6) = max_ss - u.T_margin;
end


ceq = [];
end

% motorThermal is the function for ode45 which runs off our motor
% simulation
function dydt = motorThermal(t,y,u)
% Determine motor characteristics for the RPM and SLF
% [vel_air,I,fe] = CoolDown_Handoff(u.N,u.SLF);

T_n = zeros(13,1);
T_n(1) = y(1);
T_n(2) = y(2);
T_n(3) = y(3);
T_n(4) = y(4);
T_n(5) = y(5);
T_n(6) = y(6);
T_n(7) = y(7);
T_n(8) = y(8);
T_n(9) = y(9);
T_n(10) = y(10);
T_n(11) = y(11);
T_n(12) = y(12);
T_n(13) = y(13);

[dTdt,nu_motor,Y_out,G]  = thermal_model_Joby_0427_mod(u,T_n);
dydt = zeros(13,1);
dydt(1) = dTdt(1);
dydt(2) = dTdt(2);
dydt(3) = dTdt(3);
dydt(4) = dTdt(4);
dydt(5) = dTdt(5);
dydt(6) = dTdt(6);
dydt(7) = dTdt(7);
dydt(8) = dTdt(8);
dydt(9) = dTdt(9);
dydt(10) = dTdt(10);
dydt(11) = dTdt(11);
dydt(12) = dTdt(12);
dydt(13) = dTdt(13);
assignin('base','Y_out',Y_out)
assignin('base','G',G)
% assignin('base','vel_air',vel_air)
end