% Zielfunktion ("joint range") für Optimierung in der Maßsynthese
% basierend auf Wertebereich der Gelenke des Roboters.
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
% f_jrange [1x1]
%   Zielfunktionswert: Nicht normierter Wert für die Spannweite
% 
% Siehe auch: cds_constraints.m
% 
% TODO:
% * Gleichzeitige Behandlung von Dreh- und Schubgelenken nicht sinnvoll.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [fval, fval_debugtext, debug_info, f_jrange] = cds_obj_jointrange(R, Set, Structure, Q)
debug_info = {};
fval_debugtext = '';
fval = NaN;
f_jrange = NaN;
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
II_sel = find(I_sel);
if ~any(I_sel)
  % Wenn bei seriellen Robotern die falschen Einstellungen gesetzt sind,
  % gibt es keine Lösung
  return
end
% Normiere die Gelenkwinkel. Grundlage: Gelenkwinkelgrenzen. Diese sind von
% den Einstellungen beeinflusst (nehme die nicht modifizierten
qrange_ref = Structure.qlim(:,2)-Structure.qlim(:,1);
qrange_norm = q_range./qrange_ref;
% Berechne die normierte Spannweite des kritischen Gelenks. Muss bereits
% kleiner als 1 sein, da ansonsten vorher ein Fehler aufgeworfen worden
% wäre
[q_rn_crit, Icrit] = max(qrange_norm(I_sel));
if q_rn_crit > 1
  cds_log(-1, sprintf('[cds_obj_jointrange] Normierte Gelenkwinkelspanne %1.2f>1. Darf nicht sein.', q_rn_crit));
  q_rn_crit = 1;
end
iijoint = II_sel(Icrit); % Nummer des kritischen Gelenks in allen Gelenken
% Nehme das direkt als Fitness-Wert
fval = 1e3*q_rn_crit; % Normiert auf 0 bis 1e3
f_jrange = q_range(iijoint); % Physikalischer Wert ist die nicht normierte kritische Spannweite
% Stelle lesbare Textausgabe zusammen
if Structure.Type == 2 % Paralleler Roboter
  iileg = find(iijoint >= R.I1J_LEG,1,'last'); % Nummer der Beinkette
  iilegj = iijoint - R.I1J_LEG(iileg) + 1; % Nummer des Beingelenks
  f_jrange_str = sprintf('%1.1f %s', f_jrange/R.Leg(iileg).qunitmult_eng_sci(iilegj), ...
    R.Leg(iileg).qunit_eng{iilegj});
  legstr = sprintf(' (Bein %d, Gel. %d)', iileg, iilegj);
else % Serieller Roboter
  f_jrange_str = sprintf('%1.1f %s', f_jrange/R.qunitmult_eng_sci(iijoint), ...
    R.qunit_eng{iijoint});
  legstr = '';
end
fval_debugtext = sprintf('Größte rel. Gelenkspannweite %1.1f%%. In Gel. %d%s; abs. %s.', ...
  100*q_rn_crit, iijoint, legstr, f_jrange_str);
