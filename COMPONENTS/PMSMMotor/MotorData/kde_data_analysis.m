load motor_data_kde.mat


specs = [motor_data_kde.SPECS];
const = [motor_data_kde.CONSTANTS];
%%
% Goal: Correlate motor design with motor characteristics

%% Price vs. Mass
plot([motor_data_kde.PRICE], [specs.Weight],'.b')
xlabel('Price')
ylabel('Mass')

%% Mass vs. kV
plot([const.kV],[specs.Weight],'.b')

%% kV vs Rm
plot([const.kV], [const.Rm], '.b')

%% kV + Rm vs. Mass
plot3([const.kV], [const.Rm], [specs.Weight], '.b')