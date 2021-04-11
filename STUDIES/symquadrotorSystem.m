batt = Battery('Name', 'Battery', 'N_p', symParam('N_p_batt',1), 'N_s', symParam('N_s_batt',3), 'variableV_OCV', false); % 4000mAh, 3S Default Battery, No Dynamics
prop = Propeller('Name', 'Propeller', 'k_P', symParam('k_P_prop', 0.03), 'k_T', symParam('k_T_prop', 0.05), 'D', symParam('D_prop', 0.1270), 'M', symParam('M_prop', 0.008), 'J', symParam('J_prop', 2.1075e-05));
motor = PMSMMotor('Name','Motor', 'M', symParam('M_motor',0.04), 'J', symParam('J_motor', 6.5e-6), 'K_t', symParam('K_t_motor', 0.00255), 'R_1', symParam('Rm_motor',0.117));
inv = PMSMInverter('Name','Inverter','R_1',0,'R_2',0);
%%
symQuadRotor = QuadRotor('Battery', batt, 'Propeller', prop, 'PMSMMotor', motor, 'PMSMInverter',inv);
symQuadRotor.Name = "Symbolic Quadrotor";