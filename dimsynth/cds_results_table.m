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
ResTab  = cell2table(cell(0,25), 'VariableNames', {'LfdNr', 'Name', 'Typ', ...
  'Startzeit', 'Endzeit', 'Dauer', 'Fval_Opt', 'Fval_Text', ...
  'Masse_fval', 'Masse_phys', 'Energie_fval', 'Energie_phys', ...
  'Antriebskraft_fval', 'Antriebskraft_phys','Kondition_fval', 'Kondition_phys', ...
  'Gelenkbereich_fval', 'Gelenkbereich_phys', 'Nachgiebigk_fval', 'Nachgiebigk_phys', 'Steifigk_phys', ...
  'Materialspannung', 'num_succ', 'num_fail', 'comptime_sum'});
% Zeile mit erklärenden Kommentaren zu Überschriften anhängen
Descr_Row = {'', '', '', '', '', '', 'Zielfunktion der Optimierung', '', ...
  '(normiert)', 'in kg', '(normiert)', 'in J', ...
  '(normiert)', 'in N bzw. Nm', '(normiert)', 'in Einheiten der Jacobi', ...
  '(normiert)', 'in rad', 'transl., normiert',  'in mm/N',  'in N/mm', ...
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
  % Text zu Optimierungsergebnis (insbes. Ausschlussgrund). Siehe
  % cds_vis_results, cds_constraints, cds_fitness
  f = tmp.RobotOptRes.fval;
  if f < 1e3,     fval_text = 'i.O.';
  elseif f < 1e4, fval_text = 'NB-Verl. Zielf. EO';
  elseif f < 1e5, fval_text = 'Festigkeit Segmente';
  elseif f < 1e6, fval_text = 'Kinematik-NB (Kond.)';
  elseif f < 1e3*2e3, fval_text = 'AR-Hindernis Traj.'; % ab hier aus cds_constraints
  elseif f < 1e3*3e3, fval_text = 'Bauraum-verl. Traj.';
  elseif f < 1e3*4e3, fval_text = 'Selbstkoll. Traj.';
  elseif f < 1e3*5e3, fval_text = 'Konfig. springt.';
  elseif f < 1e3*6e3, fval_text = 'Gel.-Geschw. Grenze Traj.';
  elseif f < 1e3*9e3, fval_text = 'Gel.-Pos.-Grenze Traj.';
  elseif f < 1e3*1e4, fval_text = 'Parasitäre Bew.';
  elseif f < 1e3*1e5, fval_text = 'Traj.-IK Fehler';
  elseif f < 1e3*3e5, fval_text = 'AR-Hindernis Eckpkt.';
  elseif f < 1e3*4e5, fval_text = 'Bauraum-verl. Eckpkt.';
  elseif f < 1e3*5e5, fval_text = 'Selbstkoll. Eckpkt.';
  elseif f < 1e3*1e6, fval_text = 'Gel.-Pos.-Grenze Eckpkt.';
  elseif f < 1e3*1e7, fval_text = 'Eckpkt.-IK Fehler';
  elseif f < 1e3*1e8, fval_text = 'Geom. Plausib.-Fehler 2.';
  elseif f < 1e3*1e9, fval_text = 'Geom. Plausib.-Fehler 1.';
  end
  
  % Allgemeine Daten des Optimierungsergebnisses
  Row_i = {i, Name, Structure.Type, ...
    datestr(tmp.RobotOptRes.timestamps_start_end(1),'dd.mm.yyyy HH:MM:SS'), ...
    datestr(tmp.RobotOptRes.timestamps_start_end(2),'dd.mm.yyyy HH:MM:SS'), ...
    tmp.RobotOptRes.timestamps_start_end(3), tmp.RobotOptRes.fval, fval_text};
  % Hole andere Zielfunktionen aus den Ergebnissen
  for ii = 1:6
    Row_i = [Row_i, {tmp.RobotOptRes.fval_obj_all(ii), tmp.RobotOptRes.physval_obj_all(ii)}]; %#ok<AGROW>
  end
  % Zusätzliche Nennung der Steifigkeit (Nachgiebigkeit nicht so
  % aussagekräftig)
  Row_i = [Row_i, {1/tmp.RobotOptRes.physval_obj_all(6)}]; %#ok<AGROW>
  % Überschreitung der Materialspannung
  Row_i = [Row_i, {tmp.RobotOptRes.fval_constr_all}]; %#ok<AGROW>
  % Weitere Daten
  num_succ = sum(tmp.PSO_Detail_Data.fval(:) < 1e3);
  num_fail = sum(tmp.PSO_Detail_Data.fval(:) >= 1e3);
  comptime_sum = sum(tmp.PSO_Detail_Data.comptime(~isnan(tmp.PSO_Detail_Data.comptime(:))));
  Row_i = [Row_i, {num_succ, num_fail, comptime_sum}]; %#ok<AGROW>
  % Datenzeile anhängen
  ResTab = [ResTab; Row_i]; %#ok<AGROW>
end
writetable(ResTab, restabfile, 'Delimiter', ';');
fprintf('Ergebnis-Tabelle nach %s geschrieben.\n', restabfile);