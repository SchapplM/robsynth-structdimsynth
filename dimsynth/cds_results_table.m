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
ResTab  = cell2table(cell(0,45), 'VariableNames', {'LfdNr', 'Name', 'Typ', ...
  'Beschreibung', 'Startzeit', 'Endzeit', 'Dauer', 'Fval_Opt', 'Fval_Text', ...
  'Masse_fval', 'Masse_phys', 'Energie_fval', 'Energie_phys', ...
  'Antriebskraft_fval', 'Antriebskraft_phys','Materialspannung_fval', ...
  'Materialspannung_phys','Kondition_fval', 'Kondition_phys', ...
  'Manip_fval', 'Manip_physval', 'MinSingVal_fval', 'MinSingVal_physval', ...
  'Positionsfehler_fval', 'Positionsfehler_phys', 'Gelenkbereich_fval', ...
  'Gelenkbereich_phys', 'Gelenkgrenze_fval', 'Gelenkgrenze_phys', ...
  'AntrGeschw_fval', 'AntrGeschw_phys', 'KinLaenge_fval', 'KinLaenge_phys', ...
  'Bauraum_fval', 'Bauraum_phys', 'Footprint_fval', 'Footprint_phys', ...
  'CollDist_fval', 'CollDist_phys', 'Nachgiebigk_fval', 'Nachgiebigk_phys', ...
  'Steifigk_phys', 'num_succ', 'num_fail', 'comptime_sum'});
% Zeile mit erklärenden Kommentaren zu Überschriften anhängen
Descr_Row = {'', '', '', '', '', '', '', 'Zielfunktion der Optimierung', '', ...
  '(normiert)', 'in kg', '(normiert)', 'in J', ...
  '(normiert)', 'in N bzw. Nm', '(normiert)', 'Ausnutzung der Materialgrenzen; 1=max', ...
  '(normiert)', 'in Einheiten der Jacobi', ...
  '(normiert)', 'in Einh. von J', '(normiert)', 'in Einh. von J', ...
  '(normiert)', 'in µm', '(normiert)', 'in Grad', '(normiert)', ...
  'in Grad', '(normiert)', 'in rad/s bzw. m/s', '(normiert)', 'in mm', ...
  '(normiert)', 'in m³','(normiert)', 'in m²','(normiert)', 'in mm',... % Bauraum, Fußabdruck, CollDist
  'transl., normiert',  'in mm/N',  'in N/mm', ...
  '', '', 'Rechenzeit der Fitness-Auswertungen'};
ResTab = [ResTab; Descr_Row];
% Skalierungsfaktoren der Leistungsmerkmale in der Tabelle konsistent zu
% cds_objective_plotdetails und Reihenfolge aus cds_dimsynth_robot
physval_unitmult = ones(16,1);
physval_unitmult(8) = 1e6; % Positionsfehler (in µm)
physval_unitmult(9) = 180/pi; % Gelenkbereich (in Grad)
physval_unitmult(10) = 180/pi; % Gelenkgrenze (in Grad)
physval_unitmult(12) = 1e3; % Beinkettenlänge (in mm)
physval_unitmult(15) = 1e3; % Kollisionsabstand (in mm)
% Seriell-Roboter-Datenbank für PKM-Namensgenerierung
serrob_list = load(fullfile(fileparts(which('serroblib_path_init.m')), ...
  'serrob_list.mat'), 'Names', 'AdditionalInfo');

% Alle Ergebnisse durchgehen und Tabelle erstellen
for i = 1:length(Structures)
  Structure = Structures{i};
  Name = Structure.Name;
  Beschreibung = '';
  if Structure.Type == 2
    % Erzeuge einen intuitiveren Namen für die PKM. Siehe parroblib_create_robot_class.
    [NLEG, LEG_Names]=parroblib_load_robot(Name, 0);
    I_robot = strcmp(serrob_list.Names, LEG_Names{1});
    SName_TechJoint = fliplr(regexprep(num2str(serrob_list.AdditionalInfo(I_robot,7)), ...
        {'1','2','3','4','5'}, {'R','P','C','U','S'}));
    PName_TechJoint = sprintf('%d%s', NLEG, SName_TechJoint);
    Beschreibung = PName_TechJoint;
  end
  % Ergebnisse laden. Inhalt der Datei siehe cds_dimsynth_robot.m
  resfile1 = fullfile(resmaindir, sprintf('Rob%d_%s_Endergebnis.mat', i, Name));
  resfile2 = fullfile(resmaindir, sprintf('Rob%d_%s_Details.mat', i, Name));
  if ~exist(resfile1, 'file') % Prüfe nicht die Detail-Ergebnisse resfile2. Geht auch ohne.
    warning('Ergebnis-Datei für Roboter %d/%d (%s) existiert nicht: %s', ...
      i, length(Structures), Name, resfile1);
    continue
  end
  tmp1 = load(resfile1, 'RobotOptRes');
  if exist(resfile2, 'file')
    tmp2 = load(resfile2, 'RobotOptDetails', 'PSO_Detail_Data');
  else % Platzhalter für Inhalte der fehlenden Datei
    tmp2 = struct('PSO_Detail_Data', struct('fval_mean', NaN, 'comptime', NaN));
  end
  
  % Text zu Optimierungsergebnis (insbes. Ausschlussgrund). Siehe
  % cds_constraints, cds_constraints_traj, cds_fitness, cds_vis_results
  % Strafterme aus den NB-Funktionen werden in cds_fitness erhöht.
  % Die Zuordnung erfolgt mit "<=", da die Strafterme aus dem Bereich
  % (0,1] kommen ("1" ist also möglich) und in den Zielbereich skaliert
  % werden.
  f = mean(tmp1.RobotOptRes.fval); % Falls mehrkriteriell abfangen mit `mean`
  if     f <= 1e3,     fval_text = 'i.O.'; % ab hier aus cds_fitness.m
  elseif f <= 1e4, fval_text = 'NB-Verl. Zielf.';
  elseif f <= 1e5, fval_text = 'NB-Verl. Zielf. EO';
  elseif f <= 1e6, fval_text = 'Festigkeit Segmente';
  elseif f <= 1e7, fval_text = 'Kinematik-NB (Kond.)';
  elseif f <= 1e4*1.1e3, fval_text = 'Kinematik-NB (Kond.,traj-constr)'; % ab hier aus cds_constraints_traj.m
  elseif f <= 1e4*2e3, fval_text = 'AR-Hindernis Traj.';
  elseif f <= 1e4*3e3, fval_text = 'Bauraum-verl. Traj.';
  elseif f <= 1e4*4e3, fval_text = 'Selbstkoll. Traj.';
  elseif f <= 1e4*5e3, fval_text = 'Konfig. springt.';
  elseif f <= 1e4*6e3, fval_text = 'Beschl.-Geschw. Grenze Traj.';
  elseif f <= 1e4*7e3, fval_text = 'Gel.-Geschw. Grenze Traj.';
  elseif f <= 1e4*7.2e3, fval_text = 'Schubzylinder Länge (symm) Traj.';
  elseif f <= 1e4*7.4e3, fval_text = 'Schubzylinder Länge Traj.';
  elseif f <= 1e4*7.5e3, fval_text = 'Gel.-Pos.-Grenze Traj. (symm)';
  elseif f <= 1e4*8e3, fval_text = 'Gel.-Pos.-Spannw. Traj. (symm)';
  elseif f <= 1e4*9e3, fval_text = 'Gel.-Pos.-Spannw. Traj.';
  elseif f <= 1e4*1e4, fval_text = 'Parasitäre Bew.';
  elseif f <= 1e4*2e4, fval_text = 'Traj.-IK Fehler (Beschl. 3T2R)';
  elseif f <= 1e4*3e4, fval_text = 'Traj.-IK Fehler (Geschw. 3T2R)';
  elseif f <= 1e4*4e4, fval_text = 'Traj.-IK Fehler (Pos. 3T2R)';
  elseif f <= 1e4*5e4, fval_text = 'Traj.-IK Fehler (Sing. Beinkette)';
  elseif f <= 1e4*6e4, fval_text = 'Traj.-IK Fehler (Sing. PKM)';
  elseif f <= 1e4*1e5, fval_text = 'Traj.-IK Fehler';
  elseif f <= 1e4*2e5, fval_text = 'AR-Hindernis Eckpkt.'; % ab hier aus cds_constraints.m
  elseif f <= 1e4*3e5, fval_text = 'Bauraum-verl. Eckpkt.';
  elseif f <= 1e4*4e5, fval_text = 'Selbstkoll. Eckpkt.';
  elseif f <= 1e4*4.25e5, fval_text = 'Schubzylinder Länge Eckpkt. (symm)';
  elseif f <= 1e4*4.5e5, fval_text = 'Schubzylinder Länge Eckpkt.';
  elseif f <= 1e4*5e5, fval_text = 'Gestelldurchmesser Eckpkt.';
  elseif f <= 1e4*6e5, fval_text = 'Gel.-Pos.-Grenze Eckpkt.';
  elseif f <= 1e4*7e5, fval_text = 'Gel.-Pos.-Spannweite Eckpkt.';
  elseif f <= 1e4*8e5, fval_text = 'Jacobi-Schwellwert Eckpkt.';
  elseif f <= 1e4*9e5, fval_text = 'Jacobi-Singularität Eckpkt.';
  elseif f <= 1e4*1e6, fval_text = 'Seriell-Singularität Eckpkt.';
  elseif f <= 1e4*1e7, fval_text = 'Eckpkt.-IK Fehler';
  elseif f <= 1e4*1e8, fval_text = 'Geom. Plausib.-Fehler 2.';
  elseif f <= 1e4*1e9, fval_text = 'Geom. Plausib.-Fehler 1.';
  else,                fval_text = 'Nicht definierter Fall';  
  end
  
  % Allgemeine Daten des Optimierungsergebnisses
  Row_i = {i, Name, Structure.Type, Beschreibung, ...
    datestr(tmp1.RobotOptRes.timestamps_start_end(1),'dd.mm.yyyy HH:MM:SS'), ...
    datestr(tmp1.RobotOptRes.timestamps_start_end(2),'dd.mm.yyyy HH:MM:SS'), ...
    tmp1.RobotOptRes.timestamps_start_end(3), f, fval_text};
  % Hole andere Zielfunktionen aus den Ergebnissen
  for ii = 1:length(tmp1.RobotOptRes.fval_obj_all)
    Row_i = [Row_i, {tmp1.RobotOptRes.fval_obj_all(ii), physval_unitmult(ii)*...
      tmp1.RobotOptRes.physval_obj_all(ii)}]; %#ok<AGROW>
  end
  % Zusätzliche Nennung der Steifigkeit (Nachgiebigkeit nicht so
  % aussagekräftig). Steifigkeit ist letzter Eintrag.
  Row_i = [Row_i, {1/tmp1.RobotOptRes.physval_obj_all(end)}]; %#ok<AGROW>
  % Weitere Daten
  num_succ = sum(tmp2.PSO_Detail_Data.fval_mean(:) <= 1e3);
  num_fail = sum(tmp2.PSO_Detail_Data.fval_mean(:) > 1e3 & ... % Nebenbedingungen verletzt
    ~isinf(tmp2.PSO_Detail_Data.fval_mean(:))); % "inf" ist Marker für vorzeitigen Abbruch.
  comptime_sum = sum(tmp2.PSO_Detail_Data.comptime(~isnan(tmp2.PSO_Detail_Data.comptime(:))));
  Row_i = [Row_i, {num_succ, num_fail, comptime_sum}]; %#ok<AGROW>
  % Datenzeile anhängen
  ResTab = [ResTab; Row_i]; %#ok<AGROW>
end
writetable(ResTab, restabfile, 'Delimiter', ';');
cds_log(1, sprintf('[cds_results_table] Ergebnis-Tabelle nach %s geschrieben.', restabfile));
