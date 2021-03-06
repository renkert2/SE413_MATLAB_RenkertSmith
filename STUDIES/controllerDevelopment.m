%% LinearModel from GraphModel
qr = QuadRotor();
%%
[ssMod, x0, u0, y0] = calcLinearModel(qr); % Obtain linearized model at hover
ssMod.InputName = 'u';
ssMod.OutputName = 'T';
gain = dcgain(ssMod);
dom_pole = min(abs(eigs(ssMod.A)));
ssMod_simple = tf(gain, [(1/dom_pole), 1]);

% % t = 0:0.01:5;
% % u = ones(size(t))./gain;
% % lsim(ssMod,ssMod_simple,u,t)
% ssMod_simple_ss = ss(ssMod_simple);
% f_g_lin = @(x_g,u_g) ssMod_simple_ss.A*x_g + ssMod_simple_ss.B*u_g;
% g_g_lin = @(x_g) ssMod_simple_ss.C*x_g;

%% Body Model
% u(T) = T-F_g = thrust - gravitational force
m = getProp(qr.extrinsicProps, 'Mass'); % kg

% States: X = [y; dy/dt]
A = [0 1;0 0];
B = [0; 1/m];
C = [1 0];
D = [0];

body = ss(A,B,C,D);

%step(body)
%% F This - PID
plant = series(ssMod_simple,body);

opts = pidtuneOptions('DesignFocus','disturbance-rejection');
[pidController, pidinfo] = pidtune(plant, 'PID', opts);
K_PID = [pidController.Kp, pidController.Ki, pidController.Kd];

clsys = feedback(series(pidController,plant),1);
step(clsys)

%% Controller Model
% Computes states [y int(y) diff(y)]

Ac = [0 0 1; 1 0 0; 0 0 0];
Bc = [0; 0; 1/m];
Cc = eye(3);
Dc = [0]; 

C_ss = ss(Ac,Bc,Cc,Dc);
step(C_ss)

f_c = @(x_c,u_c) Ac*x_c + Bc*u_c;
g_c = @(x_c,u_c) Cc*x_c + Dc*u_c;

%% All Together Now
syms r(t)
%r(t) = 1;
r(t) = 1-exp(-t);
int_r(t) = int(r,t);
dot_r(t) = diff(r,t);
ddot_r(t) = diff(dot_r(t),t);
r_ = matlabFunction(r);
int_r_ = matlabFunction(int_r);
dot_r_ = matlabFunction(dot_r);
ddot_r_ = matlabFunction(ddot_r);
r_PID = @(t) [r_(t); int_r_(t); dot_r_(t)];

% Indices of x_g, x_c in composite x = [x_g; x_c]
i_x_g = 1:qr.Nx;
i_x_c = i_x_g(end) + (1:size(Ac,1));

x_0 = zeros(i_x_c(end),1);
x_0(i_x_g) = x0;
x_0(i_x_g(1)) = 1;
x_0(i_x_c) = [0;0;0];

%%
[t,x] = ode23tb(@(t,x) f_sys_cl(t,x, i_x_g, i_x_c, K_PID, r_PID,  qr.f_sym_, qr.g_sym_, f_c, u0, y0), [0 100], x_0);
x_c = x(:,i_x_c);
y = x_c(:,1);
plot(t,y,t,r_(t));
title("Tracking Performance");
legend(["System", "Reference"]);
xlabel("t")
ylabel("Height (m)")


function x_dot = f_sys_cl(t,x, i_x_g, i_x_c, K_PID, r_PID, f, g, f_c, u0, y0)
    x_g = x(i_x_g);
    x_c = x(i_x_c);
    
    e = r_PID(t)-x_c;
    u = K_PID*e + u0;
    
    dot_x_g = f(x_g, u);
    
    y = g(x_g,u);
    T = y(end) - y0(end);
    dot_x_c = f_c(x_c, T);
    
    x_dot = [dot_x_g; dot_x_c];    
end



