batt = Battery('N_p', symParam('N_p_batt',1), 'V_OCV_nominal', symParam('V_OCV_batt', 3.7), 'variableV_OCV',true);
prop = Propeller('J',symParam('J_prop', 4.4e-5), 'k_P', symParam('k_P_prop', 0.0411), 'k_T', symParam('k_T_prop', 0.0819), 'D', symParam('D_prop',0.1780));
motor = PMSMMotor('L',symParam('L_motor', 1.17e-4),'J',symParam('J_motor', 6.5e-6), 'K_t', symParam('K_t_motor', 0.00255), 'R_1', symParam('R_motor', 0.117));
%%
symQuadRotor = QuadRotor('Battery', batt, 'Propeller', prop, 'PMSMMotor', motor);
symQuadRotor.Name = "Symbolic Quadrotor";