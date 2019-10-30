% TODO: Aktuell wird einfach die größte Antriebskraft minimiert. Das
% funktioniert nur bei symmetrischer Aktuierung. Ansonsten werden Kräfte
% mit Momenten vergleichen (nicht sinnvoll).

function [fval, fval_debugtext, debug_info] = cds_obj_minactforce(TAU)
debug_info = {};

tau_a_max_per_actuator = max(abs(TAU));
tau_a_max = max(tau_a_max_per_actuator);
f_actforce_norm = 2/pi*atan((tau_a_max)/100); % Normierung auf 0 bis 1; 620 ist 0.9. TODO: Skalierung ändern
fval = 1e3*f_actforce_norm; % Normiert auf 0 bis 1e3
% TODO: Einheit aus R.tauunit_sci auslesen
fval_debugtext = sprintf('Antriebskräfte max. [%s] N/Nm', disp_array(tau_a_max_per_actuator, '%1.2f'));
