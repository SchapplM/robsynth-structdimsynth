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
ResTab  = cell2table(cell(0,15), 'VariableNames', {'LfdNr', 'Name', 'Typ', ...
  'Fval_Opt', ...
  'Masse_fval', 'Masse_phys', 'Energie_fval', 'Energie_phys', ...
  'Antriebskraft_fval', 'Antriebskraft_phys','Kondition_fval', 'Kondition_phys', ...
  'num_succ', 'num_fail', 'comptime_sum'});

% Alle Ergebnisse durchgehen und Tabelle erstellen
for i = 1:length(Structures)
  Structure = Structures{i};
  Name = Structures{i}.Name;
  tmp = load(fullfile(resmaindir, ...
    sprintf('Rob%d_%s_Endergebnis.mat', i, Name)), 'RobotOptRes', 'Set', 'Traj', 'PSO_Detail_Data');
  % Allgemeine Daten des Optimierungsergebnisses
  Row_i = {i, Name, Structure.Type, tmp.RobotOptRes.fval};
  % Hole andere Zielfunktionen aus den Ergebnissen
  for ii = 1:4
    Row_i = [Row_i, {tmp.RobotOptRes.fval_obj_all(ii), tmp.RobotOptRes.physval_obj_all(ii)}]; %#ok<AGROW>
  end
  % Weitere Daten
  num_succ = sum(tmp.PSO_Detail_Data.fval(:) < 1e3);
  num_fail = sum(tmp.PSO_Detail_Data.fval(:) >= 1e3);
  comptime_sum = sum(tmp.PSO_Detail_Data.comptime(:));
  Row_i = [Row_i, {num_succ, num_fail, comptime_sum}]; %#ok<AGROW>
  % Datenzeile anhängen
  ResTab = [ResTab; Row_i]; %#ok<AGROW>
end
writetable(ResTab, restabfile, 'Delimiter', ';');
fprintf('Ergebnis-Tabelle nach %s geschrieben.\n', restabfile);