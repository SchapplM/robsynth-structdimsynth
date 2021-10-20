% Prüfe die Reproduzierbarkeit aller Zwischenergebnisse einer Maßsynthese
% Wenn keine Programmfehler vorliegen, müssten sich alle Fitness-Werte
% exakt reproduzieren lassen.
% 
% Eingabe:
% OptName
%   Name des Ordners der Maßsynthese, die nachträglich ausgewertet wird.
%   Der Ordner muss im Verzeichnis results in diesem Repo liegen.
% RobName (optional)
%   Name des Roboters, für den die Ergebnisse nachgerechnet werden sollen.
%   Auch mehrere Roboter (als 1xN Cell-array) möglich.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function cds_check_results_reproducability(OptName, RobName)

%% Optimierung laden
resdir = fullfile(fileparts(which('structgeomsynth_path_init.m')), 'results');
resdir_opt = fullfile(resdir, OptName);
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
if nargin < 2
  RobNames = Structures_Names;
elseif isa(RobName, 'cell')
  RobNames = RobName;
else
  RobNames = {RobName};
end
RobNames = unique(RobNames);
ReproStatsTab = cell2table(cell(0,7), 'VariableNames', ...
  {'RobNr', 'Name', 'Partikel', 'ResOpt', 'ResRepro', 'ErrMax', 'Status'});
%% Roboter auswerten und Nachrechnen der Fitness-Funktion
for i = 1:length(RobNames)
  RobName = RobNames{i};
  RobNr = find(strcmp(RobName,Structures_Names),1,'first');
  fprintf('Untersuche Reproduzierbarkeit für Rob %d/%d: %s\n', i, length(RobNames), RobName);
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
  if ~isempty(PSO_Detail_Data)
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
    % Variable PSO_Detail_Data nachträglich erzeugen
    PSO_Detail_Data = struct('fval', NaN(size(fval_all,1),size(fval_all,2),2));
    PSO_Detail_Data.fval(:,:,1) = fval_all;
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
    fprintf('Reproduktion Partikel Nr. %d (Gen. %d, Ind. %d):\n', jj, k_gen, k_ind);
    f2_jj = cds_fitness(R,Set,Traj,Structure_jj,p_jj,p_desopt_jj);
    test_f = f_jj - f2_jj;
    rescode = 0;
    if any(abs(test_f) > 1e-4) && f_jj(1)<1e3 || ... % Bei Erfolg feine Toleranz
        any(abs(test_f) > 1e-1) && f_jj(1)>1e3 % Bei ZB-Verletzung gröbere Toleranz
      warning(['Fitness-Wert zu Partikel Nr. %d (Gen. %d, Ind. %d) nicht ', ...
        'reproduzierbar. In Optimierung (%s): [%s]. Neu: [%s]. Diff.: [%s]'], ...
        jj, k_gen, k_ind, disp_array(Set.optimization.objective), ...
        disp_array(f_jj', '%1.4f'), disp_array(f2_jj', '%1.4f'), ...
        disp_array(test_f', '%1.4e'));
      rescode = 1;
      if all(abs(f_jj) < 1e3) && any(abs(f2_jj) > 1e3)
        rescode = 2;
        warning('Vorher i.O., jetzt n.i.O.');
      end
    end
    % Auswertungs-Tabelle schreiben
    ReproStatsTab = [ReproStatsTab; {i, RobName, jj, f_jj(1), f2_jj(1), ...
      max(abs(test_f)), rescode}]; %#ok<AGROW>
    writetable(ReproStatsTab, fullfile(resdir_opt, 'reproducability_stats.csv'), 'Delimiter', ';');
  end
end
