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
ResTab  = cell2table(cell(0,22), 'VariableNames', {'LfdNr', 'Name', 'Typ', ...
  'Startzeit', 'Endzeit', 'Dauer', 'Fval_Opt', ...
  'Masse_fval', 'Masse_phys', 'Energie_fval', 'Energie_phys', ...
  'Antriebskraft_fval', 'Antriebskraft_phys','Kondition_fval', 'Kondition_phys', ...
  'Nachgiebigk_fval', 'Nachgiebigk_phys', 'Steifigk_phys', ...
  'Materialspannung', 'num_succ', 'num_fail', 'comptime_sum'});
% Zeile mit erklärenden Kommentaren zu Überschriften anhängen
Descr_Row = {'', '', '', '', '', '', 'Zielfunktion der Optimierung', ...
  '(normiert)', 'in kg', '(normiert)', 'in J', ...
  '(normiert)', 'in N bzw. Nm', '(normiert)', 'in Einheiten der Jacobi', ...
  'transl., normiert',  'in mm/N',  'in N/mm', ...
  'Ausnutzung der Materialgrenzen; 1=max', '', '', 'Rechenzeit der Fitness-Auswertungen'};
ResTab = [ResTab; Descr_Row];
  
% Alle Ergebnisse durchgehen und Tabelle erstellen
for i = 1:length(Structures)
  Structure = Structures{i};
  Name = Structures{i}.Name;
  % Ergebnisse laden. Inhalt der Datei siehe cds_dimsynth_robot.m
  resfile = fullfile(resmaindir, sprintf('Rob%d_%s_Endergebnis.mat', i, Name));
  if ~exist(resfile, 'file')
    warning('Ergebnis-Datei für Roboter %d (%s) existiert nicht: %s', i, Name, resfile);
    continue
  end
  tmp = load(resfile, 'RobotOptRes', 'Set', 'Traj', 'PSO_Detail_Data');
  % Allgemeine Daten des Optimierungsergebnisses
  Row_i = {i, Name, Structure.Type, ...
    datestr(tmp.RobotOptRes.timestamps_start_end(1),'dd.mm.yyyy HH:MM:SS'), ...
    datestr(tmp.RobotOptRes.timestamps_start_end(2),'dd.mm.yyyy HH:MM:SS'), ...
    tmp.RobotOptRes.timestamps_start_end(3), tmp.RobotOptRes.fval};
  % Hole andere Zielfunktionen aus den Ergebnissen
  for ii = 1:5
    Row_i = [Row_i, {tmp.RobotOptRes.fval_obj_all(ii), tmp.RobotOptRes.physval_obj_all(ii)}]; %#ok<AGROW>
  end
  % Zusätzliche Nennung der Steifigkeit (Nachgiebigkeit nicht so
  % aussagekräftig)
  Row_i = [Row_i, {1/tmp.RobotOptRes.physval_obj_all(5)}]; %#ok<AGROW>
  % Überschreitung der Materialspannung
  Row_i = [Row_i, {tmp.RobotOptRes.fval_constr_all}]; %#ok<AGROW>
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