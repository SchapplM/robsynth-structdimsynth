% Zielfunktion ("objective function") für Optimierung in der Maßsynthese
% basierend auf Materialspannung des Roboters für gegebene Trajektorie.
% Die Erreichung der Streckgrenze der Strukturteile.
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% data_dyn
%   Struktur mit Dynamik-Eigenschaften des Roboters als Zeitreihe über die
%   Trajektorie. Felder:
%   Wges
%     Schnittkräfte in allen Segmenten des Roboters. Format siehe
%     cds_obj_dependencies_regmult bzw. cds_obj_dependencies
% Jinvges
%   Zeilenweise (inverse) Jacobi-Matrizen des Roboters (für PKM). Bezogen
%   auf vollständige Gelenkgeschwindigkeiten und Plattform-Geschw. (in
%   x-Koordinaten, nicht: Winkelgeschwindigkeit)
% Q
%   Gelenkwinkel-Trajektorie
% Traj_0
%   Roboter-Trajektorie (EE) bezogen auf Basis-KS des Roboters
% 
% Ausgabe:
% fval [1x1]
%   Zielfunktionswert, der im PSO-Algorithmus minimiert wird
% fval_debugtext [char]
%   Zeile mit Hinweistext, der bei PSO nach Fitness-Berechnung ausgegeben wird
% debug_info [cell]
%   Zusatz-Informationen, die im Debug-Bild des Roboters angezeigt werden
% f_maxstrengthviol [1x1]
%   Grad der Ausnutzung der Materialgrenzen:
%   1=Grenzwert gerade so erfüllt; 10=Grenzwert zehnfach überschritten.
%   (Ohne Berücksichtigung des Sicherheitsfaktors)
% 
% Siehe auch: cds_constr_yieldstrength.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [fval, fval_debugtext, debug_info, f_maxstrengthviol] = cds_obj_materialstress(R, Set, data_dyn, Jinv_ges, Q, Traj_0)
debug_info = {};
% Aufruf der Funktion für die Materialspannung als Nebenbedingung
[fval_ys, constrvioltext, f_maxstrengthviol] = cds_constr_yieldstrength(R, Set, data_dyn, Jinv_ges, Q, Traj_0);
% Fallunterscheidung für Erfolg (Linear) / kein Erfolg (gesättigt mit atan).
if fval_ys > 0
  % Grenze aus Set.optimization.safety_link_yieldstrength überschritten. 
  % Normiere auf Bereich 1e2 bis 1e3.
  fval = fval_ys / 1e2; % ursprünglich im Bereich 1e4 bis 1e5.
  fval_debugtext = constrvioltext;
else
  % Keine Grenze überschritten. Nehme die prozentuale Nutzung der Streck-
  % grenze (inklusive Sicherheitsfaktor) als Gütefunktion. Normiere auf 0 bis 1e2
  fval = 1e2*f_maxstrengthviol/Set.optimization.safety_link_yieldstrength;
  fval_debugtext = sprintf('Materialbelastungsgrenze zu %1.0f%% erreicht.', fval);
end