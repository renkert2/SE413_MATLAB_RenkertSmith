batt = Battery('N_p', symParam('N_p_batt',1));
%%
qr = QuadRotor('Battery', batt);
qr.Name = "Quadrotor";
%%
np_vals = 0.1:0.05:2;
flight_times = zeros(size(np_vals));
flight_times_simple = flight_times;
ft_simple_func = matlabFunction(flightTime_Simple(qr);

for i = 1:numel(np_vals)
    qr.updateSymParamVals(np_vals(i));
    flight_times(i) = flightTime(qr);
    flight_times_simple(i) = flightTime_Simple(qr);
end

%%
plot(np_vals, flight_times_nointerp)

%% 
opts = optimoptions('fmincon','Display','iter-detailed', 'DiffMinChange',0.001, 'PlotFcn','optimplotx');
[x,fval,exitflag,output] = fmincon(@(x) -flight_time(x,qr), 1, [],[],[],[],0.1,2,[],opts);
%%
[x,fval,exitflag,output] = ga(@(x) -flight_time(x,qr), 1, [],[],[],[],0.1,2);

function f = flight_time(x, qr)
    qr.updateSymParamVals(x);
    f = flightTime(qr);
end