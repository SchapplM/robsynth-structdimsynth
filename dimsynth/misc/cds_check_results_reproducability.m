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
  'eval_plots', {{}}, ... % Liste von Plots, die für jedes Partikel erstellt werden. Siehe Eingabe figname in cds_vis_results_figures.
  'results_dir', [], ... % Alternatives Verzeichnis zum Laden der Ergebnisse
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
ReproStatsTab_empty = cell2table(cell(0,9), 'VariableNames', ...
  {'RobNr', 'Name', 'Partikel', 'ResOpt', 'ResRepro', 'ErrMax_Rel', ...
  'ErrMax_Rel_q0set', 'WithDetails', 'Status'});
ReproStatsTab = ReproStatsTab_empty;
%% Roboter auswerten und Nachrechnen der Fitness-Funktion
for i = 1:length(RobNames)
  ReproStatsTab_Rob = ReproStatsTab_empty;
  RobName = RobNames{i};
  fprintf('Untersuche Reproduzierbarkeit für Rob %d/%d: %s\n', i, length(RobNames), RobName);
  % Finde die Roboter-Nummern zu diesem Namen. Es können mehrere parallele
  % Durchläufe mit dem gleichen Roboter gemacht worden sein. Dann nehme alle.
  RobNr_all = find(strcmp(RobName,Structures_Names));
  for RobNr = RobNr_all(:)'
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
      p_desopt_all = RobotOptRes.desopt_pval_pareto;
    else % Einkriteriell
      pval_all = RobotOptRes.p_val(:)';
      fval_all = RobotOptRes.fval;
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
    % TODO: Berücksichtige Structure_jj.q0_traj, falls nicht reproduzierbar.
    [k_gen, k_ind] = cds_load_particle_details(PSO_Detail_Data, f_jj);
    if ~isempty(PSO_Detail_Data) && isfield(PSO_Detail_Data, 'q0_ik')
      q0 = PSO_Detail_Data.q0_ik(k_ind,:,k_gen)';
    else
      q0 = RobotOptRes.q0_pareto(jj,:)';
    end
    fprintf('Reproduktion Rob. %d Partikel Nr. %d/%d (Gen. %d, Ind. %d):\n', ...
      RobNr, jj, length(I), k_gen, k_ind);
    [f2_jj, ~, Q, QD, QDD, TAU] = cds_fitness(R,Set,Traj,Structure_jj,p_jj,p_desopt_jj);
    test_f2_abs = f_jj - f2_jj;
    test_f2_rel = test_f2_abs ./ f_jj;
    test_f3_rel = NaN(size(test_f2_rel)); % Initialisierung
    rescode = 0; % Kein Fehler
    if any(abs(test_f2_rel) > 1e-2) % Fehler 1%
      warning(['Fitness-Wert zu Partikel Nr. %d (Gen. %d, Ind. %d) nicht ohne q0 ', ...
        'reproduzierbar. In Optimierung (%s): [%s]. Neu: [%s]. Diff.: [%s]%%'], ...
        jj, k_gen, k_ind, disp_array(Set.optimization.objective), ...
        disp_array(f_jj', '%1.3e'), disp_array(f2_jj', '%1.3e'), ...
        disp_array(1e2*test_f2_rel', '%1.2f'));
      % Versuche erneut mit vorgegebenen Gelenkwinkeln aus den
      % Detail-Ergebnissen
      if ~isempty(q0)
        % Trage Gelenkwinkel ein, damit Trajektorien-Prüfung erzwungen wird
        Structure_jj.q0_traj = q0;
        % Zusätzlich eintragen als Referenz-Winkel, damit es in Eckpunkt-
        % Prüfung auch genutzt wird, auch wenn es nicht von alleine
        % gefunden wird.
        if R.Type == 0 % Seriell
          R.qref = q0;
        else
          for iii = 1:R.NLEG
            R.Leg(iii).qref = q0(R.I1J_LEG(iii):R.I2J_LEG(iii));
          end
        end
        [f3_jj, ~, Q, QD, QDD, TAU] = cds_fitness(R,Set,Traj,Structure_jj,p_jj,p_desopt_jj);
      else
        f3_jj = NaN(size(f2_jj));
      end
      test_f3_abs = f_jj - f3_jj;
      test_f3_rel = test_f3_abs ./ f_jj;
      rescode = NaN; %#ok<NASGU> % Muss überschrieben werden
      if all(abs(f_jj) < 1e3) && any(abs(f2_jj) > 1e3) % vorher i.O., beim 2. mal n.i.O.
        if all(~isnan(f3_jj)) && all(abs(f3_jj) < 1e3)
          rescode = 2;
          warning('Vorher i.O., jetzt nur bei Vorgabe von q0 auch i.O., sonst n.i.O.');
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
    % Erstelle Auswertungsbilder für jeden Roboter
    if any(rescode == [0 1 2])
      RobData = struct('Name', RobName, 'Number', RobNr, 'ParetoNumber', jj, ...
        'Type', RobotOptRes.Structure.Type);
      RobotOptDetails = struct('Traj_Q', Q, 'Traj_QD', QD, 'Traj_QDD', QDD, ...
        'R', R, 'Dyn_Tau', TAU);
      restabfile = fullfile(resdir_opt, sprintf('%s_results_table.csv', OptName));
      ResTab = readtable(restabfile, 'Delimiter', ';');

      Set.optimization.resdir = resdir; % Verzeichnis des Clusters überschreiben
      for figname = s.eval_plots
        cds_vis_results_figures(figname, Set, Traj, RobData, ResTab, ...
          RobotOptRes, RobotOptDetails, [], struct('figure_invisible', true, ...
          'delete_figure', true));
      end
    end
    % Auswertungs-Tabelle für den Roboter schreiben
    details_available = ~isempty(PSO_Detail_Data);
    ReproStatsTab_Rob = [ReproStatsTab_Rob; {i, RobName, jj, f_jj(1), f2_jj(1), ...
      max(abs(test_f2_rel)), max(abs(test_f3_rel)), details_available, rescode}]; %#ok<AGROW>
    writetable(ReproStatsTab_Rob, fullfile(resdir_opt, sprintf('Rob%d_%s', RobNr, RobName), ...
      sprintf('Rob%d_%s_reproducability_stats.csv', RobNr, RobName)), 'Delimiter', ';');
    % In Gesamt-Tabelle anhängen und diese auch schreiben. Dadurch viele
    % Schreibzugriffe auf die Datei, aber auch bei Abbruch gefüllt.
    ReproStatsTab = [ReproStatsTab; ReproStatsTab_Rob]; %#ok<AGROW>
    writetable(ReproStatsTab, fullfile(resdir_opt, 'reproducability_stats.csv'), 'Delimiter', ';');
  end
  end
end
