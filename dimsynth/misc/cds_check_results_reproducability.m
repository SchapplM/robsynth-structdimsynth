% Prüfe die Reproduzierbarkeit aller Zwischenergebnisse einer Maßsynthese
% Wenn keine Programmfehler vorliegen, müssten sich alle Fitness-Werte
% exakt reproduzieren lassen. Ausnahmen können durch Zufallszahlen und
% Abweichungen der Algorithmen bei verschiedenen CPU-Architekturen bestehen
% 
% Eingabe:
% OptName
%   Name des Ordners der Maßsynthese, die nachträglich ausgewertet wird.
%   Der Ordner muss im Verzeichnis results in diesem Repo liegen.
% RobName (optional)
%   Name des Roboters, für den die Ergebnisse nachgerechnet werden sollen.
%   Auch mehrere Roboter (als 1xN Cell-array) möglich.
% s_in (optional)
%   Einstellungen zur Durchführung der Reproduzierbarkeitsstudie.
%   Felder: Siehe Quelltext
% 
% Schreibt Tabelle: reproducability_stats.csv
% (In Gesamt-Ergebnis-Ordner und in Unterordner für jeden Roboter)
% 
% Siehe auch: cds_paretoplot_buttondownfcn

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function cds_check_results_reproducability(OptName, RobName, s_in)

%% Eingabe prüfen
s = struct( ...
  'update_template_functions', false, ... % Aktualisieren die Matlab-Funktionen
  'figure_invisible', true, ... % Unsichtbar erzeugen zum Speichern ohne Fokus-Klau
  'fval_check_lim', [0, inf], ... % untere und obere Grenzen für die Prüfung der Funktionswerte
  'eval_plots', {{}}, ... % Liste von Plots, die für jedes Partikel erstellt werden. Siehe Eingabe figname in cds_vis_results_figures.
  'results_dir', '', ... % Alternatives Verzeichnis zum Laden der Ergebnisse
  'isoncluster', false, ... % Falls auf Cluster, muss der parpool-Zugriff geschützt werden
  'parcomp_maxworkers', 1, ... % Maximale Anzahl an Parallelinstanzen. Standardmäßig ohne Parfor
  'Set_mod', struct(), ... % Einstellungs-Struktur aus cds_settings_defaults zum Feld-weise Überschreiben der geladenen Einstellungen.
  'only_merge_tables', false, ... % Aufruf nur zum Zusammenführen bestehender Tabellen für einzelne Roboter
  'only_use_stored_q0', true, ... % Versuche nicht mit neuen Zufallswerten die Gelenkwinkel neu zu generieren, sondern nehme die gespeicherten.
  'only_from_pareto_front', true); % bei false werden alle Partikel geprüft, bei true nur die besten
if nargin < 3
  s_in = s;
end
for f = fields(s_in)'
  if isfield(s, f{1})
    s.(f{1}) = s_in.(f{1});
  else
    error('Feld "%s" aus s_in kann nicht übergeben werden', f{1});
  end
end
if isempty(s.results_dir)
  resdir = fullfile(fileparts(which('structgeomsynth_path_init.m')), 'results');
  resdir_opt = fullfile(resdir, OptName);
else
  resdir_opt = s.results_dir;
  [resdir, optfolder] = fileparts(resdir_opt);
  if ~strcmp(optfolder, OptName)
    error(['Der Ordnername der Optimierung heißt lokal anders, als in der ', ...
      'Datei: %s vs %s. Das gibt Probleme beim Speichern der Bilder. Abbruch.'], ...
      optfolder, OptName);
  end
end
assert(isa(s.eval_plots, 'cell'), 'Eingabefeld eval_plots muss cell array sein');
% Eindeutigen Namen für diesen Durchlauf des Versuchs der Reproduktion.
% Es kann auf verschiedenen Rechnern ein unterschiedliches Ergebnis
% rauskommen, je nach CPU-Architektur (für Zufallszahlen) oder Programmversion
repro_name = ['_', datestr(now,'yyyymmdd_HHMMSS'), '_', getComputerName()];

%% Optimierung laden
if ~exist(resdir_opt, 'file')
  warning('Ergebnis-Ordner %s existiert nicht.', resdir_opt);
  return
end
setfile = fullfile(resdir_opt, sprintf('%s_settings.mat', OptName));
if ~exist(setfile, 'file')
  warning('Einstellungs-Datei %s existiert nicht.', setfile);
  return
end
d3 = load(setfile, 'Set', 'Structures', 'Traj');
Structures = d3.Structures;
Set = cds_settings_update(d3.Set);
Set.general.isoncluster = false; % Falls auf Cluster durchgeführt, jetzt Einstellungen für lokale Auswertung
% Überschreibe die Einstellungen
for f = fields(s.Set_mod)'
  assert(isa(s.Set_mod.(f{1}), 'struct'), 'Eingabe s.Set_mod muss wiederum Struktur sein');
  for g = fields(s.Set_mod.(f{1}))'
    assert(isfield(Set.(f{1}), g{1}), sprintf('Eingabe s.Set_mod.%s hat Feld %s, aber nicht Struktur Set', f{1}, g{1}));
    Set.(f{1}).(g{1}) = s.Set_mod.(f{1}).(g{1});
  end
end
Set.optimization.resdir = resdir; % Verzeichnis des Clusters überschreiben
Set.optimization.optname = OptName;

Traj = d3.Traj;

Structures_Names = cell(1,length(Structures));
for i = 1:length(Structures)
  Structures_Names{i} = Structures{i}.Name;
end
if nargin < 2 || isempty(RobName)
  RobNames = Structures_Names;
elseif isa(RobName, 'cell')
  RobNames = RobName;
else
  RobNames = {RobName};
end
  
RobNames = unique(RobNames);
vn = {'RobNr', 'Name', 'Partikel'};
for i = 1:length(Set.optimization.objective)
  vn = [vn, sprintf('physval_%s', Set.optimization.objective{i})]; %#ok<AGROW> 
end
vn = [vn, 'ResOpt', 'ResRepro', 'ResRepro_q0set', ...
  'ErrMax_Rel', 'ErrMax_Rel_q0set', 'WithDetails', 'Status'];
ReproStatsTab_empty = cell2table(cell(0,length(vn)), 'VariableNames', vn);

%% Einstellungen zum Zusammenfassen der Tabellen
s_merge = struct('resdir_opt', resdir_opt, 'Structures', {Structures}, 'repro_names', {{}});
if s.only_merge_tables
  merge_tables(s_merge);
  return;
end

%% Roboter auswerten und Nachrechnen der Fitness-Funktion
if s.update_template_functions
  for i = 1:length(RobNames)
    Type = -1;
    for j = 1:length(Structures)
      if strcmp(Structures{j}.Name, RobNames(i))
        Type = Structures{j}.Type;
        break;
      end
    end
    if Type == 0
      serroblib_update_template_functions(RobNames(i));
    elseif Type == 2
      parroblib_update_template_functions(RobNames(i));
    else
      error('Fehler bei Typ-Bestimmung des Roboters %s', RobNames(i));
    end
  end
end
% Starten des ParPools über Wrapper-Funktion
Set_dummy = struct('general', struct('isoncluster', s.isoncluster, ...
  'parcomp_maxworkers', s.parcomp_maxworkers));
cds_log(); % Log-Funktion zurücksetzen
parfor_numworkers = cds_start_parpool(Set_dummy);
% Alle Roboter durchgehen (Aufteilung so, dass Roboter nicht auf mehrere
% parfor-Iterationen verteilt werden.
parfor (i = 1:length(RobNames), parfor_numworkers)
  if parfor_numworkers > 0
    set(0, 'defaultfigureposition', [1 1 1920 1080]);
    set(0, 'defaultfigureunits', 'pixels');
  end
  ReproStatsTab_Rob = ReproStatsTab_empty;
  RobName = RobNames{i};
  fprintf('Untersuche Reproduzierbarkeit für Rob %d/%d: %s\n', i, length(RobNames), RobName);
  % Finde die Roboter-Nummern zu diesem Namen. Es können mehrere parallele
  % Durchläufe mit dem gleichen Roboter gemacht worden sein. Dann nehme alle.
  RobNr_all = find(strcmp(RobName,Structures_Names));
  for RobNr = RobNr_all(:)'
  csvfilename = ''; % Initialisierung für Parfor-Warnung
  Structure = Structures{RobNr};
  resfile1 = fullfile(resdir_opt, sprintf('Rob%d_%s_Endergebnis.mat', ...
    RobNr, RobName));
  resfile2 = fullfile(resdir_opt, sprintf('Rob%d_%s_Details.mat', ...
    RobNr, RobName));
  if ~exist(resfile1, 'file')
    warning('Ergebnis-Datei %s existiert nicht.', resfile1);
    continue
  end
  d1 = load(resfile1, 'RobotOptRes');
  RobotOptRes = d1.RobotOptRes;
  % Prüfe Filter zum vorzeitigen Überspringen aufgrund der Funktionswerte
  % Dann muss die nächste Datei nicht geladen werden
  if s.only_from_pareto_front 
    if ~isempty(RobotOptRes.fval_pareto) % mehrkriteriell
      I_inlim = RobotOptRes.fval_pareto > s.fval_check_lim(1) & ...
                RobotOptRes.fval_pareto < s.fval_check_lim(2);
    else % einkriteriell
      I_inlim = RobotOptRes.fval > s.fval_check_lim(1) & ...
                RobotOptRes.fval < s.fval_check_lim(2);
    end
    if ~any(all(I_inlim,2))
      continue
    end
  end
  if exist(resfile2, 'file')
    d2 = load(resfile2, 'PSO_Detail_Data');
    PSO_Detail_Data = d2.PSO_Detail_Data;
  else
    PSO_Detail_Data = [];
  end
  %% Fitness-Wert nachrechnen
  if ~isempty(PSO_Detail_Data) && ~s.only_from_pareto_front
    % Detail-Ergebnisse verfügbar. Rechne alle Partikel nach.
    ngen = size(PSO_Detail_Data.pval,3);
    nind = size(PSO_Detail_Data.pval,1);
    pval_all = NaN(ngen*nind, size(PSO_Detail_Data.pval,2));
    fval_all = NaN(ngen*nind, size(PSO_Detail_Data.fval,2));
    p_desopt_all = NaN(ngen*nind, size(PSO_Detail_Data.desopt_pval,2));
    for igen = 1:ngen
      for iind = 1:nind
        pval_all((igen-1)*nind+iind,:) = PSO_Detail_Data.pval(iind,:,igen);
        fval_all((igen-1)*nind+iind,:) = PSO_Detail_Data.fval(iind,:,igen);
        p_desopt_all((igen-1)*nind+iind,:) = PSO_Detail_Data.desopt_pval(iind,:,igen);
      end
    end
  else
    % Nur Endergebnis verfügbar. Rechne diese Partikel nach.
    if ~isempty(RobotOptRes.fval_pareto) % Mehrkriteriell
      pval_all = RobotOptRes.p_val_pareto;
      fval_all = RobotOptRes.fval_pareto;
      physval_all = RobotOptRes.fval_pareto;
      p_desopt_all = RobotOptRes.desopt_pval_pareto;
    else % Einkriteriell
      pval_all = RobotOptRes.p_val(:)';
      fval_all = RobotOptRes.fval;
      physval_all = RobotOptRes.fval;
      p_desopt_all = RobotOptRes.desopt_pval(:)';
    end
    if isempty(PSO_Detail_Data)
      % Variable PSO_Detail_Data nachträglich erzeugen
      PSO_Detail_Data = struct('fval', NaN(size(fval_all,1),size(fval_all,2),2));
      PSO_Detail_Data.fval(:,:,1) = fval_all;
    end
  end
  % Debug: Zusätzliche Bilder
  % Set.general.plot_details_in_fitness = 1e10;
  % Initialisiere die Roboter-Klasse
  [R, Structure] = cds_dimsynth_robot(Set, Traj, Structure, true);
  % Sortiere die Fitness-Werte aufsteigend. Fange mit den besten für
  % Kriterium 1 an (links in 2D-Pareto-Diagramm)
  [~,I] = sortrows(fval_all, 1:size(fval_all,2));
  for jj = I(:)'
    p_jj = pval_all(jj,:)';
    f_jj = fval_all(jj,:)';
    fphys_jj = physval_all(jj,:)';
    if any(f_jj < s.fval_check_lim(1)) || any(f_jj > s.fval_check_lim(2)), continue; end
    p_desopt_jj = p_desopt_all(jj,:)';
    Structure_jj = Structure;
    if any(isnan(p_desopt_jj))
      p_desopt_jj = [];
    else
      % Keine erneute Entwurfsoptimierung, also auch keine Regressorform notwendig.
      % Direkte Berechnung der Dynamik, falls für Zielfunktion notwendig.
      Structure_jj.calc_dyn_act = Structure.calc_dyn_act | Structure.calc_dyn_reg;
      Structure_jj.calc_spring_act = Structure.calc_spring_act | Structure.calc_spring_reg;
      Structure_jj.calc_spring_reg = false;
      Structure_jj.calc_dyn_reg = false;
    end
    [k_gen, k_ind] = cds_load_particle_details(PSO_Detail_Data, f_jj);
    if ~isempty(PSO_Detail_Data) && isfield(PSO_Detail_Data, 'q0_ik')
      q0 = PSO_Detail_Data.q0_ik(k_ind,:,k_gen)';
    else
      q0 = RobotOptRes.q0_pareto(jj,:)';
    end
    fprintf('Reproduktion Rob. %d Partikel Nr. %d/%d (Gen. %d, Ind. %d):\n', ...
      RobNr, jj, length(I), k_gen, k_ind);
    f3_jj = NaN(length(f_jj), 1); % Initialisierung zum Eintragen in Tabelle
    if ~s.only_use_stored_q0
      cds_fitness(); % Persistente Variablen löschen (falls nicht in parfor)
      [f2_jj, ~, Q, QD, QDD, TAU] = cds_fitness(R,Set,Traj,Structure_jj,p_jj,p_desopt_jj);
      test_f2_abs = f_jj - f2_jj;
    else
      f2_jj = NaN(length(f_jj), 1);
      Q = []; QD = []; QDD = []; TAU = []; % für parfor-Warnung
      test_f2_abs = NaN(size(f2_jj));
    end
    test_f2_rel = test_f2_abs ./ f_jj;
    test_f3_rel = NaN(size(test_f2_rel)); % Initialisierung
    rescode = 0; % Kein Fehler
    if any(abs(test_f2_rel) > 1e-2) || s.only_use_stored_q0 % Fehler 1%
      if ~s.only_use_stored_q0
        warning(['Fitness-Wert zu Partikel Nr. %d (Gen. %d, Ind. %d) nicht ohne q0 ', ...
          'reproduzierbar. In Optimierung (%s): [%s]. Neu: [%s]. Diff.: [%s]%%'], ...
          jj, k_gen, k_ind, disp_array(Set.optimization.objective), ...
          disp_array(f_jj', '%1.3e'), disp_array(f2_jj', '%1.3e'), ...
          disp_array(1e2*test_f2_rel', '%1.2f'));
      end
      % Versuche erneut mit vorgegebenen Gelenkwinkeln aus den
      % Detail-Ergebnissen (ohne zusätzlich höherer Anzahl Versuche)
      Set_tmp = Set;
      Set_tmp.optimization.pos_ik_tryhard_num = -200; % nur auf q0 aufbauen
      if ~isempty(q0)
        % Trage Gelenkwinkel ein, damit Trajektorien-Prüfung erzwungen wird
        Structure_jj.q0_traj = q0;
        % Zusätzlich eintragen als Referenz-Winkel, damit es in Eckpunkt-
        % Prüfung auch genutzt wird, auch wenn es nicht von alleine
        % gefunden wird.
        R.update_qref(q0);
      end
      % Breche Optimierung nach dem Prüfen der ersten i.O.-Konfiguration
      % ab. Das sollte q0 sein. Sonst Fehler mit Reproduzierbarkeit.
      Set_tmp.optimization.obj_limit = 1e3*ones(length(Set.optimization.objective),1);
      cds_fitness(); % Variablen zurücksetzen, damit obj_limit unabhängig ausgewertet wird
      [f3_jj, ~, Q, QD, QDD, TAU] = cds_fitness(R,Set_tmp,Traj,Structure_jj,p_jj,p_desopt_jj);
      if isempty(Q)
        warning('Logik-Fehler. Zurückgegebenes Q ist leer')
      elseif any(abs(Q(1,:)'-q0)>1e-6)
        warning('Es wurde nicht der gespeicherte Anfangswert gewählt, sondern ein anderer.');
      end
      test_f3_abs = f_jj - f3_jj;
      test_f3_rel = test_f3_abs ./ f_jj;
      rescode = NaN; %#ok<NASGU> % Muss überschrieben werden
      if all(abs(f_jj) < 1e3) && (any(abs(f2_jj) > 1e3) || any(isnan(f2_jj))) % vorher i.O., beim 2. mal n.i.O.
        if all(~isnan(f3_jj)) && all(abs(f3_jj) < 1e3)
          if ~s.only_use_stored_q0
            rescode = 2;
            warning('Vorher i.O., jetzt nur bei Vorgabe von q0 auch i.O., sonst n.i.O.');
          else
            rescode = 0; % Kein Fehler. Besser kann es mit dieser Einstellung nicht werden
          end
        elseif all(~isnan(f3_jj)) 
          rescode = 3;
          warning('Vorher i.O., jetzt mit und ohne q0 n.i.O.');
        else
          rescode = 4;
          warning('Vorher i.O., jetzt n.i.O.');
        end
      elseif all(abs(f_jj) >= 1e3) && ~any(isnan(test_f3_rel)) % vorher n.i.O., q0 liegt vor
        if any(abs(test_f3_rel) > 1e-2) % Fehler 1%
          rescode = 5;
          warning('Vorher n.i.O., Auch mit q0 nicht exakt reproduzierbar');
        else % Kein großer Fehler bei Reproduktion mit q0
          rescode = 6;
          warning('Vorher n.i.O., Mit q0 exakt reproduzierbar, ohne nicht reproduzierbar');
        end
      else
        rescode = 1;
        warning('Wert nicht genau reproduzierbar');
      end
    end
    % Auswertungs-Tabelle für den Roboter schreiben
    details_available = ~isempty(PSO_Detail_Data);
    ReproStatsTab_Rob = [ReproStatsTab_Rob; [i, RobName, jj, ...
      num2cell(fphys_jj'), f_jj(1), f2_jj(1), f3_jj(1), ...
      max(abs(test_f2_rel)), max(abs(test_f3_rel)), details_available, rescode]]; %#ok<AGROW>
    csvfilename = fullfile(resdir_opt, sprintf('Rob%d_%s', RobNr, RobName), ...
      sprintf('Rob%d_%s_reproducability_stats%s.csv', RobNr, RobName, repro_name));
    mkdirs(fileparts(csvfilename)); % Falls Ordner nicht existiert.
    writetable(ReproStatsTab_Rob, csvfilename, 'Delimiter', ';');
    % Erstelle Auswertungsbilder für jeden Roboter
    if any(rescode == [0 1 2])
      RobData = struct('Name', RobName, 'Number', RobNr, 'ParetoNumber', jj, ...
        'Type', RobotOptRes.Structure.Type);
      RobotOptDetails = struct('Traj_Q', Q, 'Traj_QD', QD, 'Traj_QDD', QDD, ...
        'R', R, 'Dyn_Tau', TAU);
      restabfile = fullfile(resdir_opt, sprintf('%s_results_table.csv', OptName));
      opts = detectImportOptions(restabfile,'NumHeaderLines',2);
      opts.VariableNamesLine = 1;
      opts.VariableDescriptionsLine = 2;
      ResTab = readtable(restabfile, opts);
      for kk = 1:length(s.eval_plots)
        cds_vis_results_figures(s.eval_plots{kk}, Set, Traj, RobData, ResTab, ...
          RobotOptRes, RobotOptDetails, [], struct('figure_invisible', ...
          s.figure_invisible, 'delete_figure', s.figure_invisible));
      end
    end
  end
  fprintf('Tabelle für Rob %d geschrieben: %s\n', RobNr, csvfilename);
  end
end
% Alle Tabellen zusammenführen
s_merge.repro_names = {repro_name};
merge_tables(s_merge);
end

function merge_tables(s_merge)
% Zusammenfassen der Tabellen für jeden Roboter einzeln
% Bestimme alle verschiedenen durchgeführten Reproduzierbarkeits-Studien
if isempty(s_merge.repro_names)
  repro_names = {};
  for i = 1:length(s_merge.Structures)
    csvres = dir(fullfile(s_merge.resdir_opt, sprintf('Rob%d_%s', i, ...
      s_merge.Structures{i}.Name), '*_reproducability_stats*.csv'));
    for j = 1:length(csvres)
      [tokens, ~] = regexp(csvres(j).name, '_reproducability_stats([\w]*).csv', 'tokens', 'match'); % 
      if isempty(repro_names)
        repro_names = tokens{1}(1);
      elseif ~isempty(strcmp(repro_names, tokens{1}{1}))
        repro_names = [repro_names, tokens{1}{1}]; %#ok<AGROW> 
      end
    end
  end
else
  repro_names = s_merge.repro_names;
end
% Erzeuge für jede Reproduktions-Auswertung eine Zusammenfassung
ReproStatsTab = [];
for k = 1:length(repro_names)
  repro_name = repro_names{k};
  for i = 1:length(s_merge.Structures)
    csvfilename = fullfile(s_merge.resdir_opt, sprintf('Rob%d_%s', i, s_merge.Structures{i}.Name), ...
      sprintf('Rob%d_%s_reproducability_stats%s.csv', i, s_merge.Structures{i}.Name, repro_name));
    if ~exist(csvfilename, 'file'), continue; end
    ReproStatsTab_Rob = readtable(csvfilename, 'Delimiter', ';');
    if isempty(ReproStatsTab)
      ReproStatsTab = ReproStatsTab_Rob;
    else
      ReproStatsTab = [ReproStatsTab; ReproStatsTab_Rob]; %#ok<AGROW> 
    end
  end
  if isempty(ReproStatsTab)
    fprintf('Keine Tabelle mit Reproduktions-Informationen zu schreiben\n')
    continue
  end
  % Speichere die Gesamt-Tabelle für alle Roboter einer Repro-Auswertung
  csvfilename_all = fullfile(s_merge.resdir_opt, sprintf( ...
    'reproducability_stats%s.csv', repro_name));
  writetable(ReproStatsTab, csvfilename_all, 'Delimiter', ';');
  fprintf('Tabelle mit Reproduktions-Informationen geschrieben: %s\n', csvfilename_all);
end
end
