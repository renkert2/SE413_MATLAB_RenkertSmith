table_path = "C:\Users\prenk\Box\ARG_Research\PropellerData\propDataTable.csv";
propTable = readtable(table_path, 'TextType', 'string', 'ReadVariableNames', true);
propTable.Properties.VariableNames = ["Name", "Diameter", "Pitch", "Path"];
%%
propStruct = table2struct(propTable);
for i = 1:numel(propStruct)
    path = propStruct(i).Path;
    data = readtable(path);
    propStruct(i).meanCt = mean(table2array(data(:,2)));
    propStruct(i).meanCp = mean(table2array(data(:,3)));
    propStruct(i).Static.Data = data;
end

%%
save('propStruct.mat', 'propStruct')

%%
load('propStruct.mat','propStruct')
%% Lookup Vals
ps_filtered = propStruct([propStruct.Diameter] < 1 & [propStruct.Pitch] < 1);
diameter = [ps_filtered.Diameter]';
pitch = [ps_filtered.Pitch]';
ct = [ps_filtered.meanCt]';
cp = [ps_filtered.meanCp]';

%%
ct_interp = scatteredInterpolant(diameter,pitch,ct, 'natural');
ct_fit = fit([diameter pitch], ct, 'Poly33');

cp_interp = scatteredInterpolant(diameter,pitch,cp, 'natural');
cp_fit = fit([diameter pitch], cp, 'Poly33');



%% Create Valid Boundary
% bound = boundary(diameter, pitch, 0);
% 
% test_point = [0 .25 10; 0 .1 10];
% [d_min, x_d_min, y_d_min] = p_poly_dist(test_point(1,:), test_point(2,:), diameter(bound), pitch(bound), true);
% 
% dist_func = @(D,P) p_poly_dist(D, P, diameter(bound), pitch(bound), true);

bound = Boundary([diameter pitch]);
plot(bound)
%%


%%
% propFits.Interp.ct = ct_interp;
% propFits.Interp.cp = cp_interp;
% propFits.Fit.ct = ct_fit;
% propFits.Fit.cp = cp_fit;
% propFits.Boundary.points = [diameter(bound) pitch(bound)];
% propFits.Boundary.distance_func = dist_func;
% 
% save propFits.mat propFits

PF_Aero = propAeroFit();
Data.Diameter = diameter;
Data.Pitch = pitch;
Data.ct = ct;
Data.cp = cp;
PF_Aero.Data = Data;
Interp.ct = ct_interp;
Interp.cp = cp_interp;
PF_Aero.Interp = Interp;
Fit.ct = ct_fit;
Fit.cp = cp_fit;
PF_Aero.Fit = Fit;
PF_Aero.Boundary = bound;
PF_Aero.PropStruct = propStruct;

save PF_Aero.mat PF_Aero

%% 
diam_bkpt = linspace(min(diameter), max(diameter), 100);
pitch_bkpt = linspace(min(pitch), max(pitch), 100);
[diam_grid, pitch_grid] = meshgrid(diam_bkpt, pitch_bkpt);

pitch_lb = @(d) 0.5842*d - 0.0342;
pitch_ub = @(d) 0.6684*d + 0.1022;

figure(1)
plot(ct_fit, [diameter, pitch], ct)
zlim([0 max(ct)])
hold on
fplot(pitch_lb, [0 0.45], '-r')
fplot(pitch_ub, [0 0.45], '-r')
hold off
title('Propeller Thrust Coefficient')
xlabel('Diameter (m)')
ylabel('Pitch (m)')
zlabel('C_T')


figure(2)
plot(cp_fit, [diameter, pitch], cp)
zlim([0 max(cp)])
hold on
fplot(pitch_lb, [0 0.45], '-r')
fplot(pitch_ub, [0 0.45], '-r')
hold off
title('Propeller Power Coefficient')
xlabel('Diameter (m)')
ylabel('Pitch (m)')
zlabel('C_P')
