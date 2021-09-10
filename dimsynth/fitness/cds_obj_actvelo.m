% Zielfunktion ("objective function") für Optimierung in der Maßsynthese
% basierend auf der Geschwindigkeit der Antriebe des Roboters.
% Die Geschwindigkeit wird in einen normierten Zielfunktionswert übersetzt.
% Dieses Kriterium ist nur bei gleichartigen Antriebsgelenken für
% verschiedene Kinematiken vergleichbar.
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% QD
%   Gelenkgeschwindigkeiten des Roboters (für PKM auch passive Gelenke)
% 
% Ausgabe:
% fval [1x1]
%   Zielfunktionswert, der im PSO-Algorithmus minimiert wird
% fval_debugtext [char]
%   Zeile mit Hinweistext, der bei PSO nach Fitness-Berechnung ausgegeben wird
% debug_info [cell]
%   Zusatz-Informationen, die im Debug-Bild des Roboters angezeigt werden
% qD_a_max [1x1]
%   Physikalischer Wert, der dem Zielfunktionswert zugrunde liegt
%   Hier: Maximale Antriebsgeschwindigkeit

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-09
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [fval, fval_debugtext, debug_info, qD_a_max] = cds_obj_actvelo(R, QD)
debug_info = {};

if R.Type == 0
  qD_a_max_per_actuator = max(abs(QD));
else
  qD_a_max_per_actuator = max(abs(QD(:,R.I_qa)));
end
qD_a_max = max(qD_a_max_per_actuator);
% Normierung auf 0 bis 1; 2pi (1 Umdr. pro Sekunde) ist 0.8.
f_actvel_norm = 2/pi*atan(qD_a_max/2); 
fval = 1e3*f_actvel_norm; % Normiert auf 0 bis 1e3
fval_debugtext = sprintf('Antriebsgeschw. max. [%s] {m,rad}/s.', ...
  disp_array(qD_a_max_per_actuator, '%1.2f'));
