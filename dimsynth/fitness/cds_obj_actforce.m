% Zielfunktion ("objective function") für Optimierung in der Maßsynthese
% basierend auf der maximal notwendigen Antriebskraft des Roboters.
% Die max. Antriebskraft wird in einen normierten Zielfunktionswert übersetzt
% 
% Eingabe:
% TAU
%   Alle Antriebsmomente (in den aktiven Gelenken)
% 
% Ausgabe:
% fval [1x1]
%   Zielfunktionswert, der im PSO-Algorithmus minimiert wird
% fval_debugtext [char]
%   Zeile mit Hinweistext, der bei PSO nach Fitness-Berechnung ausgegeben wird
% debug_info [cell]
%   Zusatz-Informationen, die im Debug-Bild des Roboters angezeigt werden
% tau_a_max [1x1]
%   Physikalischer Wert, der dem Zielfunktionswert zugrunde liegt
%   Hier: Maximale Antriebskraft aller Antriebe in N bzw. Nm

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

% TODO: Aktuell wird einfach die größte Antriebskraft minimiert. Das
% funktioniert nur bei symmetrischer Aktuierung. Ansonsten werden Kräfte
% mit Momenten vergleichen (nicht sinnvoll).

function [fval, fval_debugtext, debug_info, tau_a_max] = cds_obj_actforce(TAU)
debug_info = {};

tau_a_max_per_actuator = max(abs(TAU));
tau_a_max = max(tau_a_max_per_actuator);
f_actforce_norm = 2/pi*atan((tau_a_max)/100); % Normierung auf 0 bis 1; 620 ist 0.9. TODO: Skalierung ändern
fval = 1e3*f_actforce_norm; % Normiert auf 0 bis 1e3
% TODO: Einheit aus R.tauunit_sci auslesen
fval_debugtext = sprintf('Antriebskräfte max. [%s] N/Nm', disp_array(tau_a_max_per_actuator, '%1.2f'));
