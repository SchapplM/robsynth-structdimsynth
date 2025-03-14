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
vn = {'LfdNr', 'Name', 'Typ', ...
  'Beschreibung', 'Startzeit', 'Endzeit', 'Dauer', 'Fval_Opt', 'Fval_Text', ...
  'Masse_fval', 'Masse_phys', 'Energie_fval', 'Energie_phys', 'Leistung_fval', 'Leistung_phys', ...
  'Antriebskraft_fval', 'Antriebskraft_phys','Materialspannung_fval', ...
  'Materialspannung_phys','Kondition_fval', 'Kondition_phys', ...
  'Manip_fval', 'Manip_physval', 'MinSingVal_fval', 'MinSingVal_physval', ...
  'Positionsfehler_fval', 'Positionsfehler_phys', 'Gelenkbereich_fval', ...
  'Gelenkbereich_phys', 'Gelenkgrenze_fval', 'Gelenkgrenze_phys', ...
  'AntrGeschw_fval', 'AntrGeschw_phys', 'KinLaenge_fval', 'KinLaenge_phys', ...
  'Bauraum_fval', 'Bauraum_phys', 'Footprint_fval', 'Footprint_phys', ...
  'CollDist_fval', 'CollDist_phys', 'Nachgiebigk_fval', 'Nachgiebigk_phys', ...
  'Steifigk_phys', 'num_pareto', 'num_succ', 'num_fail', 'comptime_sum'};
ResTab  = cell2table(cell(0,length(vn)), 'VariableNames', vn);
% Zeile mit erklärenden Kommentaren zu Überschriften anhängen
Descr_Row = {'', '', '', '', '', '', '', 'Zielfunktion der Optimierung', '', ...
  '(normiert)', 'in kg', '(normiert)', 'in J', '(normiert)', 'in W', ...
  '(normiert)', 'in N bzw. Nm', '(normiert)', 'Ausnutzung der Materialgrenzen; 1=max', ...
  '(normiert)', 'in Einheiten der Jacobi', ...
  '(normiert)', 'in Einh. von J', '(normiert)', 'in Einh. von J', ...
  '(normiert)', 'in µm', '(normiert)', 'in Grad', '(normiert)', ...
  'in Grad', '(normiert)', 'in rad/s bzw. m/s', '(normiert)', 'in mm', ...
  '(normiert)', 'in m³','(normiert)', 'in m²','(normiert)', 'in mm',... % Bauraum, Fußabdruck, CollDist
  'transl., normiert',  'in mm/N',  'in N/mm', ...
  '', '', '', 'Rechenzeit der Fitness-Auswertungen'};
ResTab = [ResTab; Descr_Row];
% Skalierungsfaktoren der Leistungsmerkmale in der Tabelle konsistent zu
% cds_objective_plotdetails und Reihenfolge aus cds_dimsynth_robot
Set_tmp = Set;
defstruct = cds_definitions();
Set_tmp.optimization.objective = defstruct.obj_names_all;
[~, physval_unitmult] = cds_objective_plotdetails(Set_tmp);
% Seriell-Roboter-Datenbank für PKM-Namensgenerierung
serrob_list = load(fullfile(fileparts(which('serroblib_path_init.m')), ...
  'serrob_list.mat'), 'Names', 'AdditionalInfo');

% Alle Ergebnisse durchgehen und Tabelle erstellen
for i = 1:length(Structures)
  Structure = Structures{i};
  Number = Structure.Number;
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
  resfile1 = fullfile(resmaindir, sprintf('Rob%d_%s_Endergebnis.mat', Number, Name));
  resfile2 = fullfile(resmaindir, sprintf('Rob%d_%s_Details.mat', Number, Name));
  if ~exist(resfile1, 'file') % Prüfe nicht die Detail-Ergebnisse resfile2. Geht auch ohne.
    warning('Ergebnis-Datei für Roboter %d/%d (%s) existiert nicht: %s', ...
      i, length(Structures), Name, resfile1);
    continue
  end
  try
    tmp1 = load(resfile1, 'RobotOptRes');
    RobotOptRes = tmp1.RobotOptRes;
  catch e
    warning('Fehler beim Laden der Ergebnis-Datei für Roboter %d/%d (%s): %s', ...
      i, length(Structures), Name, e.message);
    continue
  end
  if exist(resfile2, 'file')
    try
      tmp2 = load(resfile2, 'RobotOptDetails', 'PSO_Detail_Data');
    catch e
      tmp2 = [];
      warning('Fehler beim Laden der Ergebnis-Detail-Datei für Roboter %d/%d (%d/%s): %s', ...
        i, length(Structures), Number, Name, e.message);
    end
  else
    tmp2 = [];
  end
  if isempty(tmp2) % Platzhalter für Inhalte der fehlenden Datei
    tmp2 = struct('PSO_Detail_Data', struct('fval_mean', NaN, 'comptime', NaN));
  end
  PSO_Detail_Data = tmp2.PSO_Detail_Data;
  % Text zu Optimierungsergebnis (insbes. Ausschlussgrund). Siehe
  % cds_constraints, cds_constraints_traj, cds_fitness, cds_vis_results
  % Strafterme aus den NB-Funktionen werden in cds_fitness erhöht.
  % Die Zuordnung erfolgt mit "<=", da die Strafterme aus dem Bereich
  % (0,1] kommen ("1" ist also möglich) und in den Zielbereich skaliert
  % werden.
  % TODO: Diese Grenzen sind in der Datei misc/constraints_fval_limits.csv
  % abgelegt und die Datei sollte hier geladen werden.
  f = mean(RobotOptRes.fval); % Falls mehrkriteriell abfangen mit `mean`
  if     f <= 1e3,     fval_text = 'i.O.'; % ab hier aus cds_fitness.m
  elseif f <= 1e4, fval_text = 'NB-Verl. Zielf. (Antriebskraft)';
  elseif f <= 2e4, fval_text = 'NB-Verl. Zielf. EO (Masse)';
  elseif f <= 3e4, fval_text = 'NB-Verl. Zielf. EO (Antriebskraft)';
  elseif f <= 4e4, fval_text = 'NB-Verl. Zielf. EO (Steifigkeit)';
  elseif f <= 1e5, fval_text = 'NB-Verl. Zielf. EO (TODO)';
  elseif f <= 4e5, fval_text = 'EO: Festigkeit Segmente';
  elseif f <= 8e5, fval_text = 'EO: Selbstkollision';
  elseif f <= 1e6, fval_text = 'EO: Fehler (unplausible Werte)';
  elseif f <= 2e6, fval_text = 'Undefiniert';
  elseif f <= 6e6, fval_text = 'Kinematik-NB (Pos.-Fehler)';
  elseif f <= 1e7, fval_text = 'Kinematik-NB (Kond.)';
  elseif f <= 1e4*1.1e3, fval_text = 'Kinematik-NB (PosErr.,traj-constr)'; % ab hier aus cds_constraints_traj.m
  elseif f <= 1e4*1.2e3, fval_text = 'Kinematik-NB (Kond.,traj-constr)';
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
  elseif f < 1e4*1e4, fval_text = 'Parasitäre Bew.';
  elseif f == 1e4*1e4, fval_text = 'Inaktives Gelenk';
  elseif f <= 1e4*2e4, fval_text = 'Traj.-IK Fehler (Beschl. 3T2R)';
  elseif f <= 1e4*3e4, fval_text = 'Traj.-IK Fehler (Geschw. 3T2R)';
  elseif f <= 1e4*4e4, fval_text = 'Traj.-IK Fehler (Pos. 3T2R)';
  elseif f <= 1e4*5e4, fval_text = 'Traj.-IK Fehler (Sing. Beinkette)';
  elseif f <= 1e4*6e4, fval_text = 'Traj.-IK Fehler (Sing. PKM)';
  elseif f <= 1e4*1e5, fval_text = 'Traj.-IK Fehler';
  elseif f <= 1e4*2e5, fval_text = 'AR-Hindernis Eckpkt.'; % ab hier aus cds_constraints.m
  elseif f <= 1e4*3e5, fval_text = 'Bauraum-verl. Eckpkt.';
  elseif f <= 1e4*3.9e5, fval_text = 'Selbstkoll. Eckpkt.';
  elseif f <= 1e4*4e5, fval_text = 'Einbaulage nicht symmetrisch Eckpkt.';
  elseif f <= 1e4*4.25e5, fval_text = 'Schubzylinder Länge Eckpkt. (symm)';
  elseif f <= 1e4*4.5e5, fval_text = 'Schubzylinder Länge Eckpkt.';
  elseif f <= 1e4*4.9e5, fval_text = 'Beinkettenlänge Eckpkt.';
  elseif f <= 1e4*5e5, fval_text = 'Gestelldurchmesser Eckpkt.';
  elseif f <= 1e4*5.4e5, fval_text = 'Einbaulage nicht symmetrisch Eckpkt.';
  elseif f <= 1e4*5.5e5, fval_text = 'Plattform-Rotation-Grenze Eckpkt.';
  elseif f <= 1e4*6e5, fval_text = 'Gel.-Pos.-Grenze Eckpkt.';
  elseif f <= 1e4*7e5, fval_text = 'Gel.-Pos.-Spannweite Eckpkt.';
  elseif f <= 1e4*7.5e5, fval_text = 'Positionsfehler-Grenze Eckpkt.';
  elseif f <= 1e4*8e5, fval_text = 'Jacobi-Schwellwert Eckpkt.';
  elseif f <= 1e4*9e5, fval_text = 'Jacobi-Singularität Eckpkt.';
  elseif f <= 1e4*1e6, fval_text = 'Seriell-Singularität Eckpkt.';
  elseif f == 1e4*9.9e6, fval_text = 'Eckpkt.-IK Fehler (IK-Singularität)';
  elseif f <= 1e4*1e7, fval_text = 'Eckpkt.-IK Fehler'; % Sonderfall, geht eigentlich nur bis 9.8527e10
  elseif f <= 1e4*1e8, fval_text = 'Geom. Plausib.-Fehler 2.';
  elseif f <= 1e4*1e9, fval_text = 'Geom. Plausib.-Fehler 1.';
  elseif f <= 1e14,    fval_text = 'Parameter unplausibel';
  else,                fval_text = 'Nicht definierter Fall';  
  end
  
  % Allgemeine Daten des Optimierungsergebnisses:
  if any(isnan(RobotOptRes.timestamps_start_end(1)))
    datecols = {'', ''};
  else % Funktion kann in neuen Matlab-Versionen kein NaN mehr abfangen
    datecols = {
      datestr(RobotOptRes.timestamps_start_end(1),'dd.mm.yyyy HH:MM:SS'), ...
      datestr(RobotOptRes.timestamps_start_end(2),'dd.mm.yyyy HH:MM:SS')
      };
  end
  Row_i = {Number, Name, Structure.Type, Beschreibung, datecols{1}, datecols{2},...
    RobotOptRes.timestamps_start_end(3), f, fval_text};
  % Hole andere Zielfunktionen aus den Ergebnissen. TODO: Code kann
  % vereinfacht werden, wenn keine alten Daten mehr damit verarbeitet
  % werden müssen.
  if length(physval_unitmult) == length(RobotOptRes.fval_obj_all)
    for ii = 1:length(RobotOptRes.fval_obj_all)
      Row_i = [Row_i, {RobotOptRes.fval_obj_all(ii), physval_unitmult(ii)*...
        RobotOptRes.physval_obj_all(ii)}]; %#ok<AGROW>
    end
  else
    warning('Dimension von fval_obj_all aus Ergebnis ist nicht konsistent mit Toolbox-Version.');
    if isfield(RobotOptRes, 'obj_names_all')
      % Trage die Zielkriterien ein, die vorhanden sind. Der Rest wird NaN
      for ii = 1:length(physval_unitmult)
        I_inres = strcmp(defstruct.obj_names_all{ii}, RobotOptRes.obj_names_all);
        if ~any(I_inres) % Zielkriterium war nicht in Optimierung drin. Vermutlich alte Version dort.
          warning('Zielkriterium %s nicht in Ergebnisse gefunden.', defstruct.obj_names_all{ii});
          Row_i = [Row_i, {NaN, NaN}]; %#ok<AGROW>
        else
          Row_i = [Row_i, {RobotOptRes.fval_obj_all(I_inres), physval_unitmult(ii)*...
            RobotOptRes.physval_obj_all(I_inres)}]; %#ok<AGROW>
        end
      end
    else
      % Keine Zuordnung aktuell bestimmbar. Setze alles auf NaN.
      for ii = 1:length(physval_unitmult)
        Row_i = [Row_i, {NaN, NaN}]; %#ok<AGROW>
      end
    end
  end
  % Zusätzliche Nennung der Steifigkeit (Nachgiebigkeit nicht so
  % aussagekräftig). Steifigkeit ist letzter Eintrag.
  Row_i = [Row_i, {1/RobotOptRes.physval_obj_all(end)}]; %#ok<AGROW>
  % Weitere Daten
  num_pareto = size(RobotOptRes.fval_pareto,1);
  num_succ = nnz(PSO_Detail_Data.fval_mean(:) <= 1e3);
  num_fail = nnz(PSO_Detail_Data.fval_mean(:) > 1e3 & ... % Nebenbedingungen verletzt
    ~isinf(PSO_Detail_Data.fval_mean(:))); % "inf" ist Marker für vorzeitigen Abbruch.
  comptime_sum = sum(PSO_Detail_Data.comptime(~isnan(PSO_Detail_Data.comptime(:))));
  Row_i = [Row_i, {num_pareto, num_succ, num_fail, comptime_sum}]; %#ok<AGROW>
  % Datenzeile anhängen
  ResTab = [ResTab; Row_i]; %#ok<AGROW>
end
writetable(ResTab, restabfile, 'Delimiter', ';');
cds_log(1, sprintf('[results_table] Ergebnis-Tabelle nach %s geschrieben.', restabfile));
