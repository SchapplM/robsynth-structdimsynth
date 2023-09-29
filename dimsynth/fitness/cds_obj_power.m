% Zielfunktion ("objective function") für Optimierung in der Maßsynthese
% basierend auf maximaler Leistung des Roboters für gegebene Trajektorie.
% Die Leistung bestimmt sich aus Maximalgeschwindigkeit und Maximalmoment
% der Antriebe, unabhängig vom zeitlichen Vorkommen. Die Kennzahl ist daher
% eher als Auslegungsgröße zu sehen. Berücksichtige auch Anzahl Beinketten.
% Die Leistung wird in einen normierten Zielfunktionswert übersetzt
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
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
%   (summiert über alle Antriebe)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-08
% (C) Institut für Mechatronische Systeme, Leibnis Universität Hannover

function [fval, fval_debugtext, debug_info, P_max_krit] = cds_obj_power(R, Set, TAU, QD)
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
  % PKM: Annahme einer symmetrischen PKM. Maximum von Kraft und Geschw.
  % ist entscheidend für Antriebsauslegung
  if Set.optimization.obj_power.symmetric_speed_torque_limits
    P_max_krit = R.NLEG * max(TAU_max) * max(QD_max);
  else % Alte Berechnung (zur Kompatibilität): Keine Symmetrische Auslegung. 
    % ... Max. Leistung wird für jeden Antrieb für sich bestimmt (mit eigenem Getriebe).
    % ... Ist weniger plausibel für eine spätere Anwendung.
    P_max_krit = R.NLEG * max(P_max);
  end
end
f_pwr_norm = 2/pi*atan((P_max_krit)/100); % Normierung auf 0 bis 1; 620 ist 0.9
fval = 1e3*f_pwr_norm; % Normiert auf 0 bis 1e3
if R.Type == 0
  fval_debugtext = sprintf('Sum. max. Antr. Leistung %1.1f W. (%s)', P_max_krit, ...
    disp_array(P_max, '%1.1f'));
else
  if P_max_krit > 1e3, unit = 'kW'; scale = 1e3;
  else,                unit = 'W';  scale = 1;
  end
  if Set.optimization.obj_power.symmetric_speed_torque_limits
    fval_debugtext = sprintf(['Max. Antr. Leistung %1.1f %s (aus qD_max=%1.1f', ...
      ' und tau_max=%1.1f).'], P_max_krit/scale, unit, max(QD_max), max(TAU_max));
  else
    fval_debugtext = sprintf('Max. Antr. Leistung %1.1f %s (%s).', P_max_krit/scale, unit, ...
      disp_array(P_max, '%1.1f'));
  end
end
