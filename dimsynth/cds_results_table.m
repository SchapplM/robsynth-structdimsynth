% Erstelle eine Ergebnis-Tabelle zur besseren Übersicht über die Ergebnisse
% 
% Eingabe:
% Set
%   Einstellungen des Optimierungsalgorithmus
% Traj
%   Trajektorie (bezogen auf Welt-KS)
% Structures
%   Eigenschaften der Roboterstrukturen (alle an Optimierung beteiligten)
%   Siehe cds_gen_robot_list.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-11
% (C) Institut für Mechatronische Systeme, Universität Hannover

function cds_results_table(Set, Traj, Structures)
% Initialisierung
resmaindir = fullfile(Set.optimization.resdir, Set.optimization.optname);
restabfile = fullfile(resmaindir, sprintf('%s_results_table.csv', Set.optimization.optname));
ResTab  = cell2table(cell(0,4), 'VariableNames', {'LfdNr', 'Name', 'Typ', 'Fval'});

% Alle Ergebnisse durchgehen und Tabelle erstellen
for i = 1:length(Structures)
  Structure = Structures{i};
  Name = Structures{i}.Name;
  tmp = load(fullfile(resmaindir, ...
    sprintf('Rob%d_%s_Endergebnis.mat', i, Name)), 'RobotOptRes', 'Set', 'Traj');
  ResTab = [ResTab; {i, Name, Structure.Type, tmp.RobotOptRes.fval}]; %#ok<AGROW>
end
% Tabelle speichern
writetable(ResTab, restabfile, 'Delimiter', ';');