qr = QuadRotor();
s = qr.SimpleModel;
lm = qr.LinearDynamicModel;

%% Body Model
[Am,Bm,~,Dm] = calcBodyModel(qr);
Cm = eye(2);

B = ss(Am,Bm,Cm,Dm);
B.InputName = {'T'};
B.OutputName = {'Y','Ydot'};

%% PowerTrain Model
[Apt, Bpt, Cpt, Dpt] = calcLinearMatrices(qr);
pt_ss = ss(Apt,Bpt,Cpt,Dpt);
gain = Dpt-Cpt*(Apt\Bpt);
dom_pole = min(abs(eigs(Apt)));
den = [(1/dom_pole), 1];
PT = [tf(gain(1), den); tf(gain(2), den)];
PT.InputName = {'u'};
PT.OutputName = {'W','T'};

%% Inner Speed Loop
opts = pidtuneOptions('PhaseMargin', 90);
PI_speed = pidtune(PT(1),'PI', 20, opts);
PI_speed.InputName = 'e_w';
PI_speed.OutputName = 'u';

sb_speed = sumblk('e_w = r_w - W');

speed_loop = connect(sb_speed, PI_speed, PT, 'r_w', {'W', 'T'});


%%
figure(1)
step(speed_loop)

figure(2)
rlocusplot(PI_speed*PT(1))
title('Root Locus: Speed Control Loop')
hold on
poles = pole(speed_loop(1));
zeros = zero(speed_loop(1));
plot(real(poles), imag(poles), 'xr');
plot(real(zeros), imag(zeros), 'or');
hold off

%% Velocity (ydot) Loop
vel_plant = connect(speed_loop, B, 'r_w', 'Ydot');
opts = pidtuneOptions('PhaseMargin', 90);
PD_vel = pidtune(vel_plant,'PD', 10, opts);
PD_vel.InputName = 'e_v';
PD_vel.OutputName = 'r_w';

sb_vel = sumblk('e_v = r_v - Ydot');
vel_loop = connect(sb_vel, PD_vel, speed_loop, B, 'r_v',{'Y','Ydot'});


%%
figure(1)
step(vel_loop(2))
figure(2)
rlocusplot(PD_vel*vel_plant)
title('Root Locus: Climb Rate Control Loop')
hold on
poles = pole(vel_loop(2));
zeros = zero(vel_loop(2));
plot(real(poles), imag(poles), 'xr');
plot(real(zeros), imag(zeros), 'or');
hold off
%% Outer Height Loop
P_height = pidtune(vel_loop(1),'P',2);
P_height.InputName = 'e_h';
P_height.OutputName = 'r_v';

sb_height = sumblk('e_h = r_h - Y');
height_loop = connect(sb_height, P_height, vel_loop, 'r_h', 'Y');


%%
figure(1)
step(height_loop)
figure(2)
rlocusplot(P_height*vel_loop(1))
title('Root Locus: Height Control Loop')
hold on
poles = pole(height_loop);
zeros = zero(height_loop);
plot(real(poles), imag(poles), 'xr');
plot(real(zeros), imag(zeros), 'or');
hold off

%% Sqrt Control
fplot(@sqrtControl, [-10 10])
title('Square-Root Control')
xlabel('$$e_h$$','Interpreter','latex')
ylabel('$$f(e_h)$$','Interpreter','latex')


function x = sqrtControl(x)
if abs(x) > 1
    x = sign(x)*(sqrt(abs(4*x)) - 1);
end
end
