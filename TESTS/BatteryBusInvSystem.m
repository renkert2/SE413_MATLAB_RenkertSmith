%% Init
close all
addpath(genpath('COMPONENTS'));

%% Instantiate Components

battery = Battery('Name', "Battery");
dcbus = DCBus_CurrentEquivalence('Name', 'Bus', 'N_inputs',1,'N_outputs',1);
pmsminverter = PMSMInverter('Name', "Inverter");

%% Connect Components
components = [battery, dcbus, pmsminverter];

ConnectP = {[battery.Ports(1) dcbus.Ports(1)];
    [dcbus.Ports(2), pmsminverter.Ports(1)]};

g_sys = Combine(components, ConnectP);
sys_model = GraphModel(g_sys)

%% Plot Graph
plot(sys_model)

%% System Tables
% Vertex Table
names = vertcat(g_sys.Vertices.Description);
parents = vertcat(vertcat(g_sys.Vertices.Parent).Name);
types = vertcat(g_sys.Vertices.VertexType);
vertex_table = table((1:(g_sys.Nv+g_sys.Nev))', parents, names, types, 'VariableNames', ["Vertices", "Component", "Description", "Domain"])

% Edge Table
digits(4)
pflows = vpa(sys_model.P_sym);
pflows_strings = arrayfun(@string, pflows);
digits(32)

parents = vertcat(vertcat(g_sys.InternalEdges.Parent).Name);

edge_table = table((1:(g_sys.Ne - g_sys.Nee))',parents, pflows_strings, 'VariableNames', ["Edges", "Component", "PowerFlows"])

%% System Simulation

sys_model.StateNames
sys_model.InputNames
sys_model.DisturbanceNames

inputs = 2;
disturbances = zeros(4,1);
disturbances(3) = 15;
 
% %[t,x] = Simulate(sys_model, inputs, disturbances, [0 10], 'StateSelect', [1,13]);
Simulate(sys_model, inputs, disturbances, [0 1], 'StateSelect', [6 8 5]);