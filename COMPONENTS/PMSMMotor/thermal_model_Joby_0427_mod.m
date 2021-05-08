function [dTdt,nu_motor,Y_out,G]  = thermal_model_Joby_0427_mod(u,T)
% Determine current density and rpm scaling values

G.d_wire = 0.254/1000;           % AWG 30

% Outlet Temperature for heat sink calculations
Tout = u.T_amb; % 3 matches transient data
% u.poles = ceil(u.poles);
u.slots = u.poles;

% Frequencies
wr = u.wr; % mechanical angular frequency [rad/s]
u.fe = wr * u.poles/(4*pi);
% we = u.fe*2*pi; % electrical angular frequency [rad/s]

%% Model Geometry and Thermal Properties
% Uses metric units
% To create the thermal model, certain geometric and thermal properties
% are necessary

% Begin with machine geometry
G.Dw = 12e-3*u.DF;     % Slotless winding thickness
G.Dsy = 4e-3*u.DF;     % Stator yoke thickness
G.Dins = 1e-3*u.DF;    % Stator insulator thickness
G.Dh = 3e-3*u.DF;     % Thermal fin plane thickness
G.Dht = 17e-3*u.DF;    % Thermal fin teeth length
G.Wht = 2.5e-3;     % width between fin teeth
G.Tw = 1.5e-3;      % thickness of tooth
G.g = 2e-3*u.DF;            % air gap thickness

G.L = 0.128 * u.SLF;           % machine active length

G.Ro = 0.1837*u.DF;               % rotor outer radius
G.Rm = 0.173*u.DF;               % magnet outer radius

G.Rw = 0.165*u.DF;      % Winding outer radius
% Simulink does not allow adding fields after the structure has been read,
% so define all values ahead of time
G.Rsy = 0; G.Rins = 0; G.Rh = 0; G.Rht = 0; G.Rsi = 0; G.Rr = 0;G.Ww = 0;
G.Rsy = G.Rw-G.Dw;     % Stator yoke outer radius
G.Rins = G.Rsy-G.Dsy;  % Insulator layer outer radius %%%%%%%%%%%%%%%%%%%%%
G.Rh = G.Rins-G.Dins;     % Thermal fin outer radius %%%%%%%%%%%%%%%%%%%%%
G.Rht = G.Rh-G.Dh;     % Thermal fin teeth outer radius %%%%%%%%%%%%%%%%%%%%%
G.Rsi = G.Rht-G.Dht;     % Stator inner radius %%%%%%%%%%%%%%%%%%%%%

G.Rr = G.Rw + G.g;               % rotor inner radius
G.Ww = pi * (G.Rw + G.Rsy) /u.slots - G.Dw;    % Width of slot

p = u.poles/2;                     % number of pole pairs
n_fins = 2*pi*G.Rht/(G.Wht+G.Tw);       % number of cooling fins

% Calculate copper resistivity
T_ref = 20;                     % reference temperature [deg C]
alpha = 0.00393;                % temperature coefficient of resistance for copper [-/deg C]
rho_cu_ref = 1.68e-8;           % copper resistivity coupled with thermal impacts
T_wind_max = max(T(2:10));
rho_cu = rho_cu_ref * (1 + alpha*(T_wind_max - T_ref));

% Mass Properties
rho_al = 2700;
rho_fe = 7650;
rho_wind = 4705.75;
rho_mag = 7500;
mass_hs = (pi*(G.Rht^2 - G.Rsi^2) - n_fins*(G.Dht*G.Wht)) * G.L * rho_al;
mass_iron = (pi*(G.Rw^2 - G.Rins^2) - u.slots*G.Dw*G.Ww) * G.L * rho_fe;
mass_wind = (u.slots * G.Dw*G.Ww) * G.L * rho_wind;
mass_mag = pi*(G.Rm^2 - G.Rr^2) * G.L * rho_mag;
mass_rot_out = pi*(G.Ro^2 - G.Rm^2) * G.L * rho_al;
Y_out.active_mass = mass_hs + mass_iron + mass_wind + mass_mag;

% Mass moment of inertia, change units to kg mm^2
% I_mag = mass_mag/2*(G.Rm^2 + G.Rr^2)*1e6;
% I_rot_out = mass_rot_out/2*(G.Ro^2 + G.Rm^2)*1e6;
% Y_out.I_rotor = I_mag + I_rot_out;

%% Electromagnetics
% % theta = 0;                  % angle of magnetic field analysis
B_rem = 1.15;               % magnet remnant flux
mu_r = 1.04;                % magnet permeability
B_sat_sy = 2.0;             % iron yoke saturation flux
stack_fac = 0.97;           % stacking factor (out of 1)
J_s = 7.25;                 % slot current density

% D_0 is an intermediate variable used to calculate radial flux density
D_0 = 2*(1-mu_r) * ((1-mu_r)*(G.Rr/G.Rm)^(2*p) + (1 + mu_r)*(G.Rsy/G.Rm)^(2*p))...
    - 2*(1+mu_r)*((1+mu_r)+(1-mu_r)*(G.Rsy/G.Rm)^(2*p));
% B_r_fcn allows us to calculate B_r for any r and theta
B_r_fcn = @(r,theta) 4*B_rem/D_0 * p/(1-p) * (1+mu_r) * (1 - (G.Rr/G.Rm)^(p-1)) ...
    * (1 + (G.Rsy/r)^(2*p)) * (r/G.Rr)^(p-1) * cos(p * theta);
% B_t_fcn allows us to calculate B_r for any r and theta
B_t_fcn = @(r,theta) -4*B_rem/D_0 * p/(1-p) * (1+mu_r) * (1 - (G.Rr/G.Rm)^(p-1)) ...
    * (1 + (G.Rsy/r)^(2*p)) * (r/G.Rr)^(p-1) * sin(p * theta);

% B_G.Rsy is for r = stator yoke outer radius and theta = 0
B_G.Rsy = B_r_fcn((G.Rr+G.Rm)/2,0)*2;
B_G.Rw = B_r_fcn(G.Rw,0)*2;


%% Fluid Properties *
mu_table = [-50	14.56e-6;
    -25	15.88e-6;
    -15	16.4e-6;
    -10	16.65e-6;
    -5	16.9e-6;
    0	17.15e-6;
    5	17.4e-6;
    10	17.64e-6;
    15	17.89e-6;
    20	18.13e-6;
    25	18.37e-6;
    30	18.6e-6;
    40	19.07e-6;
    50	19.53e-6;
    60	19.99e-6;
    80	20.88e-6;
    100	21.74e-6;
    125	22.79e-6;
    150	23.8e-6;
    175	24.78e-6];

k_table = [-50	20.41e-3;
    -25	22.41e-3;
    -15	23.2e-3;
    -10	23.59e-3;
    -5	23.97e-3;
    0	24.36e-3;
    5	24.74e-3;
    10	25.12e-3;
    15	25.5e-3;
    20	25.87e-3;
    25	26.24e-3;
    30	26.62e-3;
    40	27.35e-3;
    50	28.08e-3];

cp_table = [-53.2	1.006e3;
    -33.2	1.006e3;
    -13.2	1.006e3;
    0	1.006e3;
    6.9	1.006e3;
    15.6	1.006e3;
    26.9	1.006e3;
    46.9	1.007e3;
    66.9	1.009e3;
    86.9	1.01e3];

nu_table = [-50	9.22e-6;
    -25	11.18e-6;
    -15	12.01e-6;
    -10	12.43e-6;
    -5	12.85e-6;
    0	13.28e-6;
    5	13.72e-6;
    10	14.16e-6;
    15	14.61e-6;
    20	15.06e-6;
    25	15.52e-6;
    30	15.98e-6;
    40	16.92e-6;
    50	17.88e-6];

rho_table = [-50	1.582;
    -25	1.422;
    -15	1.367;
    -10	1.341;
    -5	1.316;
    0	1.292;
    5	1.268;
    10	1.246;
    15	1.225;
    20	1.204;
    25	1.184;
    30	1.164;
    40	1.127;
    50	1.093];

rho_air = interp1(rho_table(:,1),rho_table(:,2),Tout);
nu_air = interp1(nu_table(:,1),nu_table(:,2),Tout);
mu_air = interp1(mu_table(:,1),mu_table(:,2),Tout);
k_air = interp1(k_table(:,1),k_table(:,2),Tout);
cp_air = interp1(cp_table(:,1),cp_table(:,2),Tout);
Pr_air = nu_air * cp_air / k_air;

%% Iron Losses

% Curve fitting coefficients for JNEX900
k_h = 0.02831;
k_e = 1.77e-7;
alpha = 1.589;
beta = 0.7441;

% Use these and the frequency to calculate specific iron loss
%     P_iron_specific = k_h*fe*B_yk^2 + k_e*fe.^2*B_yk^2;
P_iron_specific = k_h*u.fe*B_G.Rsy^(alpha+beta*B_G.Rsy) + 2*pi^2*k_e*u.fe^2*B_G.Rsy^2;
P_core = P_iron_specific * mass_iron;       % specific iron losses are W/kg, so convert mass

%% DC Winding Losses
% Estimate number of turns, parallel paths, and strands per slot
G.n_turns = floor(G.Ww/2/G.d_wire)-1;
G.n_pp = floor(G.Dw/2/G.d_wire);
N_sc = G.n_turns * G.n_pp;

% Estimate phase resistance and I^2R losses
Y_out.R_phase = (G.n_turns*G.L*rho_cu)/(G.n_pp*pi/4*G.d_wire^2);
P_w_res = 3 * Y_out.R_phase * u.I^2 / sqrt(2);

%% AC Loss predictions
%% Wire Eddy Current Losses
P_se_vol = pi * u.fe^2 * B_G.Rw^2 * G.d_wire^4 * G.L / (128 * rho_cu);
P_w_eddy = P_se_vol * u.slots * N_sc;     % losses per strand to total losses

%% Magnet Eddy Current Losses
rho_mag_ref = 1.50e-6;       %https://www.stanfordmagnets.com/characteristics-of-ndfeb-magnets.html
alpha_mag = 2e-4;
rho_mag = rho_mag_ref * (1 + alpha_mag*(max(T) - T_ref));
P_mag_dens = ((G.Rm-G.Rr)^2 * u.fe^2 * B_G.Rw^2) / (24* rho_mag);
P_mag = (pi*(G.Rm^2 - G.Rr^2)*G.L)*P_mag_dens;


%% Mechanical Losses
mu_f = 0.024;       % Bearing friction factor
P_bear = 0.5 * mu_f * 2*G.Rsi * (Y_out.active_mass*1/3 * 9.81) * wr;  
% bearing losses, estimate load as 1/3 of total motor mass

% First we calculate the inner rotor windage losses
Re_gap = wr * G.Rr * G.g / nu_air;            % reynolds number in gap of concentric cylinders
C_cm = 0.065 * (G.g/G.Rr)^0.3 * Re_gap ^ (-0.2);     % C_cm is friction coefficient
P_gap = 1/2 * pi * rho_air * wr^3 * G.Rr^4 * G.L * C_cm;      % gap losses

P_mech = P_gap + P_bear;

%% Heat Load Estimation
eff_reduce_k = 1.65;         % constant to multiply losses and thus decrease efficiency

Ploss.Pcu = (P_w_res + P_w_eddy)*eff_reduce_k;
Ploss.Piron = P_core*eff_reduce_k;
Ploss.Pwind_gap = (P_mech + P_mag)*eff_reduce_k;
% motor_pow = wr * u.Qdemand            % omega * tau
motor_pow = u.I * u.V;
nu_motor = (1 - (Ploss.Pcu + Ploss.Piron + Ploss.Pwind_gap)/motor_pow) * 100; % efficiency for 60 kW

%% Heat Transfer Coefficient Estimation, Method 1
% Currently we use the same air temperature for air gap and heat sink

a_gap = 2*G.Rw;                    % inner annulus diameter
b_gap = 2*(G.Rw + G.g);                % outer annulus diameter
Dh_gap = 2 * G.g;

air_gap_percent = 50/100;                   % percent of air velocity in air gap, ...
% 50% taken from Figure 22 of Joby Motor Design paper
v_ag_axial = u.vel_air * air_gap_percent;     % axial velocity of flow
v_ag_rot = wr * G.Rw;                       % rotational velocity of flow


%% Use Tachnibana paper for combined axial and rotational flow
v_eff = sqrt(v_ag_axial^2 + (v_ag_rot^2)/4);
Nu_gap = 0.015* (v_eff*Dh_gap/nu_air)^0.8 * (1+2.3*Dh_gap/G.L) *...
    (b_gap/a_gap)^0.45 * Pr_air^(1/3);


%% Calculate Gap HT Value
HT.h_gap = Nu_gap * k_air / Dh_gap;

%% HT HS
a_hs = G.Wht;
b_hs = G.Dht;
Dh_hs = 2*a_hs*b_hs / (a_hs + b_hs);   % Treat as fully filled rectangular duct
Re_hs = u.vel_air * Dh_hs / nu_air;      % Reynolds number of flow in heat sink

% Heat sink modeling
if Re_hs > 3000 && Re_hs < 1e5
    f_hs = (0.79 * log(Re_hs) - 1.64)^(-2);     % Darcy friction factor correlation
    Nu_hs = (f_hs / 8 * (Re_hs - 1000) * Pr_air) / (1 + 12.7 * sqrt(f_hs / 8) * ...
        (Pr_air ^ (2/3) - 1));          % nusselt number for heat sink (Gnielinki)
elseif Re_hs <= 3000 && Re_hs >= 450        % intermediate flow
    consts = [7.15866378296294e-11;2.99402219738851e-07;...
        -0.000312950880246466;5.67367561424157];
    Nu_hs = consts(1)*Re_hs^3 + consts(2)*Re_hs^2 + consts(3)*Re_hs + consts(4);
elseif Re_hs < 450     % Correlation for fully developed flow
    Nu_hs = 5.60;
else
    error_msg = sprintf("Heat sink heat Reynolds: %0.0f\n",Re_hs);
    error(error_msg)
end
HT.h_hs = Nu_hs * k_air / Dh_hs;     % convective ht coefficient for heat sink
S_hs = 2*pi*(G.Rht+G.Rsi)*(2.3/3)+2*n_fins*G.Dht;    % Heat sink total effective surface area

%% Thermal conductivity of components [W/mK]
KT.kcu = 1.8145;
KT.kins = 1;
KT.kiron = 20;
KT.kal = 201;

% Thermal equivlent circuit in 2-D: transient analysiss
% All the thermal resistances and loss components are normalized to unit
% length (1 meter) active length case.
%   Detailed explanation goes here
% Input:
% G - Machine Dimensions
% Ploss - Machine Loss Vector
% Tout - Outlet Temperature
% HT - Heat Transfer Coefficient Vector
% S_hs - Heat sink total surface area
% KT - Thermal conductivity of different materials

% L = G.L;
% n_fins = 140;
% S_hs = 2*pi*(G.Rht+G.Rsi)*(2.3/3)+2*n_fins*G.Dht;

% T_n = T.T_n;        % current model temperature
Tg = Tout;        % Gap temperature
Ths = Tout;       % Heat sink temperature

rho_res = 1200;
rho_wind = 4705.75;
rho_iron = 7650;
rho_al = 2702;

cp_res = 1000;
cp_wind = 723.25;
cp_iron = 500;
cp_al = 903;
mat_factor = 0.8;  % Material factor to match Joby motor paper

% Define thermal capacity
C = zeros(13);
C(1,1) = mat_factor*rho_wind*cp_wind*pi*(G.Rw^2-(G.Rw - G.Dw/10)^2);
C(2,2) = mat_factor*rho_wind*cp_wind*pi*((G.Rw - G.Dw/10)^2-(G.Rw - G.Dw/10*2)^2);
C(3,3) = mat_factor*rho_wind*cp_wind*pi*((G.Rw - G.Dw/10*2)^2-(G.Rw - G.Dw/10*3)^2);
C(4,4) = mat_factor*rho_wind*cp_wind*pi*((G.Rw - G.Dw/10*3)^2-(G.Rw - G.Dw/10*4)^2);
C(5,5) = mat_factor*rho_wind*cp_wind*pi*((G.Rw - G.Dw/10*4)^2-(G.Rw - G.Dw/10*5)^2);
C(6,6) = mat_factor*rho_wind*cp_wind*pi*((G.Rw - G.Dw/10*5)^2-(G.Rw - G.Dw/10*6)^2);
C(7,7) = mat_factor*rho_wind*cp_wind*pi*((G.Rw - G.Dw/10*6)^2-(G.Rw - G.Dw/10*7)^2);
C(8,8) = mat_factor*rho_wind*cp_wind*pi*((G.Rw - G.Dw/10*7)^2-(G.Rw - G.Dw/10*8)^2);
C(9,9) = mat_factor*rho_wind*cp_wind*pi*((G.Rw - G.Dw/10*8)^2-(G.Rw - G.Dw/10*9)^2);
C(10,10) = mat_factor*(rho_wind*cp_wind*pi*((G.Rw - G.Dw/10*9)^2-(G.Rw - G.Dw/10*10)^2)+ rho_iron*cp_iron*pi*((G.Rsy)^2-(G.Rsy - G.Dsy/4)^2));
C(11,11) = mat_factor*rho_iron*cp_iron*pi*((G.Rsy - G.Dsy/4)^2-(G.Rsy - G.Dsy/4*2)^2);
C(12,12) = mat_factor*rho_iron*cp_iron*pi*((G.Rsy - G.Dsy/4*2)^2-(G.Rsy - G.Dsy/4*3)^2);
C(13,13) = mat_factor*rho_iron*cp_iron*pi*((G.Rsy - G.Dsy/4*3)^2-(G.Rsy - G.Dsy/4*4)^2);

% Define thermal resistance
Rhg = 1/(HT.h_gap*pi*G.Rw*2);
Rcu1 = log(G.Rw/(G.Rw - G.Dw/10))/(KT.kcu*2*pi);
Rcu2 = log((G.Rw - G.Dw/10)/(G.Rw - G.Dw/10*2))/(KT.kcu*2*pi);
Rcu3 = log((G.Rw - G.Dw/10*2)/(G.Rw - G.Dw/10*3))/(KT.kcu*2*pi);
Rcu4 = log((G.Rw - G.Dw/10*3)/(G.Rw - G.Dw/10*4))/(KT.kcu*2*pi);
Rcu5 = log((G.Rw - G.Dw/10*4)/(G.Rw - G.Dw/10*5))/(KT.kcu*2*pi);
Rcu6 = log((G.Rw - G.Dw/10*5)/(G.Rw - G.Dw/10*6))/(KT.kcu*2*pi);
Rcu7 = log((G.Rw - G.Dw/10*6)/(G.Rw - G.Dw/10*7))/(KT.kcu*2*pi);
Rcu8 = log((G.Rw - G.Dw/10*7)/(G.Rw - G.Dw/10*8))/(KT.kcu*2*pi);
Rcu9 = log((G.Rw - G.Dw/10*8)/(G.Rw - G.Dw/10*9))/(KT.kcu*2*pi);
Rcu10 = log((G.Rw - G.Dw/10*9)/G.Rsy)/(KT.kcu*2*pi);
Rsy1 = log(G.Rsy/(G.Rsy - G.Dsy/4))/(KT.kiron*2*pi);
Rsy2 = log((G.Rsy - G.Dsy/4)/(G.Rsy - G.Dsy/4*2))/(KT.kiron*2*pi);
Rsy3 = log((G.Rsy - G.Dsy/4*2)/(G.Rsy - G.Dsy/4*3))/(KT.kiron*2*pi);
Rsy4 = log((G.Rsy - G.Dsy/4*3)/G.Rh)/(KT.kiron*2*pi);
Rins = log(G.Rh/(G.Rh-0.001))/(KT.kins*2*pi);
Rhs = 1/(HT.h_hs*S_hs);


% Thermal Resistance Matrix
Gh = zeros(13,13);
Gh(1,1) = 1/Rhg + 1/Rcu1;
Gh(1,2) = - 1/Rcu1;
Gh(2,1) = Gh(1,2);
Gh(2,2) = 1/Rcu1 + 1/Rcu2;
Gh(2,3) = - 1/Rcu2;
Gh(3,2) = Gh(2,3);
Gh(3,3) = 1/Rcu2 + 1/Rcu3;
Gh(3,4) = - 1/Rcu3;
Gh(4,3) = Gh(3,4);
Gh(4,4) = 1/Rcu3 + 1/Rcu4;
Gh(4,5) = -1/Rcu4;
Gh(5,4) = Gh(4,5);
Gh(5,5) = 1/Rcu4 + 1/Rcu5;
Gh(5,6) = - 1/Rcu5;
Gh(6,5) = Gh(5,6);
Gh(6,6) = 1/Rcu5 + 1/Rcu6;
Gh(6,7) = - 1/Rcu6;
Gh(7,6) = Gh(6,7);
Gh(7,7) = 1/Rcu6 + 1/Rcu7;
Gh(7,8) = - 1/Rcu7;
Gh(8,7) = Gh(7,8);
Gh(8,8) = 1/Rcu7 + 1/Rcu8;
Gh(8,9) = - 1/Rcu8;
Gh(9,8) = Gh(8,9);
Gh(9,9) = 1/Rcu8 + 1/Rcu9;
Gh(9,10) = - 1/Rcu9;
Gh(10,9) = Gh(9,10);
Gh(10,10) = 1/(Rcu10+Rsy1) +1/Rcu9;
Gh(10,11) = - 1/(Rcu10+Rsy1);
Gh(11,10) = Gh(10,11);
Gh(11,11) = 1/(Rcu10+Rsy1) + 1/Rsy2;
Gh(11,12) = - 1/Rsy2;
Gh(12,11) = Gh(11,12);
Gh(12,12) = 1/Rsy2 + 1/Rsy3;
Gh(12,13) = - 1/Rsy3;
Gh(13,12) = Gh(12,13);
Gh(13,13) = 1/Rsy3 + 1/(Rsy4+Rins+Rhs);

Ploss.Pcu = Ploss.Pcu/G.L;
Ploss.Piron = Ploss.Piron/G.L;
Ploss.Pwind_gap = Ploss.Pwind_gap/G.L;

% Heat Source Vector
Qh = zeros(13,1);
Qh(1) = Ploss.Pwind_gap + Tg/Rhg;
Qh(2) = Ploss.Pcu/9;
Qh(3) = Ploss.Pcu/9;
Qh(4) = Ploss.Pcu/9;
Qh(5) = Ploss.Pcu/9;
Qh(6) = Ploss.Pcu/9;
Qh(7) = Ploss.Pcu/9;
Qh(8) = Ploss.Pcu/9;
Qh(9) = Ploss.Pcu/9;
Qh(10) = Ploss.Pcu/9;
Qh(11) = Ploss.Piron/3;
Qh(12) = Ploss.Piron/3;
Qh(13) = Ploss.Piron/3 + Ths/(Rsy4+Rins+Rhs);

dTdt = inv(C)*(Qh-Gh*T);


%% Estimate torque constant
% Use curve fit constants to estimate torque constant
a = 29.47;
b = 27.96;
c = -645.9;
d = 1.231;
e = 1.416;
f = 0.03415;
g = 0.09588;
f4 = @(x,y) (a./(x+f)).^d + (b./(y+g)).^e + c;
K_v = f4(Y_out.active_mass,Y_out.R_phase)*2;
Y_out.K_tau = 60/(2*pi*K_v);
end
