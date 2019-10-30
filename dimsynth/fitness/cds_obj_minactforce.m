function [fval, fval_debugtext, debug_info] = cds_obj_minactforce(TAU)
debug_info = {};

tau_a_max_per_leg = max(abs(TAU));
tau_a_max = max(tau_a_max_per_leg);
f_actforce_norm = 2/pi*atan((tau_a_max)/100); % Normierung auf 0 bis 1; 620 ist 0.9. TODO: Skalierung ändern
fval = 1e3*f_actforce_norm; % Normiert auf 0 bis 1e3
fval_debugtext = sprintf('Antriebskräfte max. [%s]', disp_array(tau_a_max_per_leg, '%1.2f'));
