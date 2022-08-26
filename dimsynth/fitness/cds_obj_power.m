% Zielfunktion ("objective function") für Optimierung in der Maßsynthese
% basierend auf maximaler Leistung des Roboters für gegebene Trajektorie.
% Die Leistung bestimmt sich aus Maximalgeschwindigkeit und Maximalmoment
% der Antriebe, unabhängig vom zeitlichen Vorkommen. Die Kennzahl ist daher
% eher als Auslegungsgröße zu sehen
% Die Leistung wird in einen normierten Zielfunktionswert übersetzt
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% TAU
%   Alle Antriebsmomente (in den aktiven Gelenken)
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
% P_max_krit [1x1]
%   Physikalischer Wert, der dem Zielfunktionswert zugrunde liegt
%   Hier: maximale Leistung in W, gemäß implementiertem Algorithmus

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-08
% (C) Institut für Mechatronische Systeme, Leibnis Universität Hannover

function [fval, fval_debugtext, debug_info, P_max_krit] = cds_obj_power(R, TAU, QD)
debug_info = {};

% Berechne die maximale Leistung jedes Antriebs. 
TAU_max = max(abs(TAU));
if R.Type == 0
  QD_max = max(abs(QD));
else
  QD_max = max(abs(QD(:,R.I_qa)));
end
P_max = TAU_max .* QD_max;

if R.Type == 0
% Seriell: Basisnahe Antriebe haben eher größere Leistung. Kriterium daher
  % als die Summe dieser maximalen Leistungen.
  P_max_krit = sum(P_max);
else
  % PKM: Annahme einer symmetrischen PKM. Maximum ist entscheidend für
  % Antriebsauslegung
  P_max_krit = max(P_max);
end
f_pwr_norm = 2/pi*atan((P_max_krit)/100); % Normierung auf 0 bis 1; 620 ist 0.9
fval = 1e3*f_pwr_norm; % Normiert auf 0 bis 1e3
if R.Type == 0
  fval_debugtext = sprintf('Sum. max. Antr. Leistung %1.1f W. (%s)', P_max_krit, ...
    disp_array(P_max, '%1.1f'));
else
  fval_debugtext = sprintf('Max. Antr. Leistung %1.1f W (%s).', P_max_krit, ...
    disp_array(P_max, '%1.1f'));
end
