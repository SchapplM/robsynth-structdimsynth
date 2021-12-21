% Zielfunktion ("joint limit") für Optimierung in der Maßsynthese
% basierend auf festen Grenzen der Gelenke des Roboters.
% Ist sinnvoll wenn Eigenschaften vorgegebener Robotermodelle optimiert
% werden. Bei neu zu konstruierenden Robotern sollte eher das Kriterium
% cds_obj_jointrange genommen werden.
% Das Kriterium wird in einen normierten Zielfunktionswert übersetzt
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
% f_jlim [1x1]
%   Zielfunktionswert: Nicht normiert
% 
% Siehe auch: cds_obj_jointrange.m, cds_constraints_traj.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [fval, fval_debugtext, debug_info, f_jlim] = cds_obj_jointlimit(R, Set, Structure, Q)
debug_info = {};

% Relative Überschreitung der Grenzen berechnen
qlim = Structure.qlim;
% Normalisiere Gelenkwinkel auf 0...1
Q_norm = (Q - repmat(qlim(:,1)', size(Q,1), 1)) ./ ...
          repmat(qlim(:,2)'-qlim(:,1)', size(Q,1), 1);
% Normalisiere auf -0.5...+0.5. Dadurch Erkennung der Verletzung einfacher
Q_limviolA = 2*abs(Q_norm-0.5); % 0 entspricht jetzt der Mitte.
% Kriterium als Maximalwert der normierten Abweichungen von der Mitte.
% Ein Wert von 1 ist genau an Grenze.
[f_jlim1, iisample] = max(Q_limviolA);
[f_jlim, iijoint] = max(f_jlim1);
% Der Wert muss bereits kleiner als 1 sein, da ansonsten vorher ein Fehler
% aufgeworfen worden wäre
if f_jlim > 1
  cds_log(-1, sprintf('[cds_obj_jointlimit] Normierte Gelenkwinkel %1.2f>1. Darf nicht sein.', f_jlim));
end
% Nehme das direkt als Fitness-Wert
fval = 1e3*f_jlim; % Normiert auf 0 bis 1e3

% Stelle lesbare Textausgabe zusammen
if Structure.Type == 2 % Paralleler Roboter
  iileg = find(iijoint >= R.I1J_LEG,1,'last'); % Nummer der Beinkette
  iilegj = iijoint - R.I1J_LEG(iileg) + 1; % Nummer des Beingelenks
  legstr = sprintf(' (Bein %d, Gel. %d)', iileg, iilegj);
else % Serieller Roboter
  legstr = '';
end
fval_debugtext = sprintf(['Kleinster rel. Abstand zur Gelenkgrenze ', ...
  '%1.1f%%. In Gel. %d%s; Schritt %d/%d'], 100*(1-f_jlim), iijoint, ...
  legstr, iisample(iijoint), size(Q,1));
