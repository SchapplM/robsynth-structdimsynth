% Zielfunktion ("joint range") für Optimierung in der Maßsynthese
% basierend auf Wertebereich der Gelenke des Roboters.
% Die Konditionszahl wird in einen normierten Zielfunktionswert übersetzt
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Structure
%   Eigenschaften der Roboterstruktur (aus cds_gen_robot_list.m)
% Q
%   Gelenkpositionen des Roboters (für PKM auch passive Gelenke)
% 
% Ausgabe:
% fval [1x1]
%   Zielfunktionswert, der im PSO-Algorithmus minimiert wird
% fval_debugtext [char]
%   Zeile mit Hinweistext, der bei PSO nach Fitness-Berechnung ausgegeben wird
% debug_info [cell]
%   Zusatz-Informationen, die im Debug-Bild des Roboters angezeigt werden
% 
% Siehe auch: cds_constraints.m
% 
% TODO:
% * Gleichzeitige Behandlung von Dreh- und Schubgelenken nicht sinnvoll.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [fval, fval_debugtext, debug_info, f_jrange] = cds_obj_jointrange(R, Set, Structure, Q)
debug_info = {};
fval = NaN;
% Spannweite berechnen
q_range = NaN(R.NJ,1);
q_range(R.MDH.sigma==1) = diff(minmax2(Q(:,R.MDH.sigma==1)')');
q_range(R.MDH.sigma==0) = angle_range( Q(:,R.MDH.sigma==0));
% Indizes zur Auswahl der Gelenke
if Structure.Type == 2 % Paralleler Roboter
  I_active = R.I_qa;
else
  I_active = R.MDH.mu;
end
I_prismatic = R.MDH.sigma==1;

I_sel = true(R.NJ,1); % Nehme alle Gelenke
if Set.optimization.obj_jointrange.only_revolute
  I_sel(I_prismatic) = false; % Schubgelenke deaktivieren
end
if Set.optimization.obj_jointrange.only_passive
  I_sel(I_active) = false; % aktive deaktivieren
end
if ~any(I_sel)
  % Wenn bei seriellen Robotern die falschen Einstellungen gesetzt sind,
  % gibt es keine Lösung
  return
end
% Normiere die Gelenkwinkel. Grundlage: Gelenkwinkelgrenzen. Diese sind von
% den Einstellungen beeinflusst.
qrange_ref = Structure.qlim(:,2)-Structure.qlim(:,1);
qrange_norm = q_range./qrange_ref;
% Berechne die normierte Spannweite des kritischen Gelenks. Muss bereits
% kleiner als 1 sein, da ansonsten vorher ein Fehler aufgeworfen worden
% wäre
[q_rn_crit, Icrit] = max(qrange_norm(I_sel));
if q_rn_crit > 1
  warning('Normierte Gelenkwinkelspanne ist größer als 1. Darf nicht sein.');
  q_rn_crit = 1;
end
% Nehme das direkt als Fitness-Wert
fval = 1e3*q_rn_crit; % Normiert auf 0 bis 1e3
f_jrange = q_range(Icrit); % Physikalischer Wert ist die nicht normierte kritische Spannweite
fval_debugtext = sprintf('Größte rel. Gelenkspannweite: %1.2f (Gel.-Nr. %d; abs. %1.2f)', ...
  q_rn_crit, Icrit, f_jrange);