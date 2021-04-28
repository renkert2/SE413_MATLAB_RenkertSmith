% Reid Smith, Spring 2021
% Code seeks to take in iinputs Z and use design variables to optimize
% performance of coupling variables

% load('motor_data.mat')
% for i = 1:499
% if ~isnan(motor_data(i).SPECS.Maximum_RPM)
% fprintf("%1i\n",i)
% end
% end
clear
clc
close all

%% Compare model to kde data
u.T_begin = 20;    % starting temperature (landing temperature)
u.T_end = 80;       % ending temperature (take off temperature)
u.T_amb = 20;       % ambient temperature
u.T_margin = 120;   % winding threshold temperature
% u.N = Z_in.N;       % rotational speed, passed down from top level
% u.N=2000;
wr = 13e3*2*pi/60;
u.poles = 12;
u.slots = 12;
u.fe = wr * u.poles/(4*pi);
u.vel_air = 22;
u.I = 24;
u.V = 26.1;

u.SLF = 31.7/128;          % stack length factor
u.DF = 35.5/187.3;           % diameter factor
y0 = ones(13,1)*u.T_margin;

[~,~,Y_out,~]  = thermal_model_Joby_0427_mod(u,y0)

%% Run Simulation

Y_in.I_rotor = 100;
Y_in.K_tau = 0.005;
Y_in.R_phase = 0.13;
Y_in.active_mass = 0.125;

Y_out = ATC_lower_fcn(1,Y_in)

function [Y_out] = ATC_lower_fcn(Z_in,Y_in)
% u is structure of inputs
u.T_begin = 20;    % starting temperature (landing temperature)
u.T_end = 80;       % ending temperature (take off temperature)
u.T_amb = 20;       % ambient temperature
u.T_margin = 120;   % winding threshold temperature
% u.N = Z_in.N;       % rotational speed, passed down from top level
% u.N=2000;
wr = 13e3*2*pi/60;
u.poles = 14;
u.slots = 12;
% poles = 8;
u.fe = wr * u.poles/(4*pi);
u.vel_air = 22;

% u.SLF = 31.7/128;          % stack length factor
% u.DF = 35.5/187.3;           % diameter factor
u.I = 24;
u.V = 26.1;
input_cell = cell(1,2);
input_cell{1} = Y_in;
input_cell{2} = u;

% Set Torque Constant Fitting Coefficients
% f = motorDataFit()
% u.a = f.a;
% u.b = f.b;
% u.c = f.c;
% u.d = f.d;
% u.e = f.e;
% u.f = f.f;
% u.g = f.g;
% 
% tspan = [0,100];
% y0 = ones(13,1)*u.T_begin;
% 
% [t,y] = ode23tb(@(t,y) motorThermal(t,y,u), tspan, y0);
% figure
% plot(t,y)
x0 = [0.1,0.1,12];
LB = [0,0,0];
options = optimset('disp','iter','TolFun',1e-3);
[xval,fval] = fmincon(@(x)obj_low_ATC(x,input_cell),x0,[],[],[],[],LB,[],@(x)constr_low_ATC(x,input_cell),...
    options)
u.DF = xval(1);
u.SLF = xval(2);
u.poles = xval(3);
y0 = ones(13,1)*u.T_margin;
[~,~,Y_out,~]  = thermal_model_Joby_0427_mod(u,y0);

end

%% Functions

function [f_obj] = obj_low_ATC(x,input_cell)
Y_in = input_cell{1};
u = input_cell{2};
v_2 = 1;
u.DF = x(1);
u.SLF = x(2);
u.poles = x(3);
% u.V = 26.1;

% Use final temp to determine values
% tspan = [0,100];
y0 = ones(13,1)*u.T_margin;
% [~,y] = ode23tb(@(t,y) motorThermal(t,y,u), tspan, y0);
% T_n = y(end,:)';

% Run model to determine estimates
[~,~,Y_out,~]  = thermal_model_Joby_0427_mod(u,y0);
% Generate c_2 and do preliminary scaling
c_2 = [(Y_in.I_rotor - Y_out.I_rotor)/Y_in.I_rotor; ...
    (Y_in.K_tau - Y_out.K_tau)/Y_in.K_tau; ...
    (Y_in.R_phase - Y_out.R_phase)/Y_in.R_phase;...
    (Y_in.active_mass - Y_out.active_mass)/Y_in.active_mass];
f_obj = v_2*norm(c_2);
end

% Constraint function
function [c,ceq] = constr_low_ATC(x,input_cell)
Y_in = input_cell{1};
u = input_cell{2};
u.DF = x(1);
u.SLF = x(2);
u.poles = x(3);

tspan = [0,100];
y0 = ones(13,1)*u.T_begin;

% Check for geometry violations
[~,~,~,G]  = thermal_model_Joby_0427_mod(u,y0);
c(1) = G.d_wire - G.Dw;
c(2) = G.d_wire - G.Ww;
c(3) = 1 - G.n_turns;
c(4) = 1 - G.n_pp;
c(5) = 2 - x(3);

% Check for temperature violations


[~,y] = ode23tb(@(t,y) motorThermal(t,y,u), tspan, y0);
max_ss = max(y(:,end));
c(6) = max_ss - u.T_margin;


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