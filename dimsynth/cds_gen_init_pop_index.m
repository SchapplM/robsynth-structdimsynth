% Erzeuge einen Index der vorhandenen Ergebnisdateien
% Ermöglicht einen schnelleren Dateizugriff auf dem Cluster
% 
% Eingabe:
% Set
%   Einstellungen des Optimierungsalgorithmus aus cds_settings_defaults
% Structures
%   Eigenschaften aller optimierter Roboterstrukturen der Maßsynthese
% make_global_index [logical]
%   Schalter zur Erzeugung eines globalen Index für den kompletten
%   Ergebnisordner. Falls true: Eingabe Set und Structures kann leer
%   gelassen werden.
% 
% Benutzung:
% Globalen Index erstellen:
% `cds_gen_init_pop_index([], [], true);`
% 
% Siehe auch: cds_gen_init_pop

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-04
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function cds_gen_init_pop_index(Set, Structures, make_global_index)

if isempty(Set)
  resdir = fullfile(fileparts(which( ...
                  'structgeomsynth_path_init.m')), 'results');
  Set = struct('optimization', struct( ...
    'InitPopRatioOldResults', 1, ...
    'result_dirs_for_init_pop', {{resdir}}, ...
    'resdir', resdir));
end

if Set.optimization.InitPopRatioOldResults == 0
  % Es sollen keine alten Ergebnisse geladen werden. Kein Durchsuchen der
  % Ordner notwendig.
  return
end
if nargin < 3
  make_global_index = false;
end
t1 = tic();
% Dateiname für Index-Datei bestimmen
if make_global_index
  filename_idx = fullfile(Set.optimization.resdir, 'index_results.mat');
else
  resdir_main = fullfile(Set.optimization.resdir, Set.optimization.optname);
  mkdirs(fullfile(resdir_main, 'tmp'));
  filename_idx = fullfile(resdir_main, 'tmp', 'old_results.mat');
end


% Bestimme die Namen der zu prüfenden Roboter für den Index
RobNames = {};
for k = 1:length(Structures)
  if Structures{k}.Type == 2 % PKM: Wähle auch Ergebnisse mit anderen Koppelgelenk-Anordnungen
    [~, ~, ~, ~, ~, ~, ~, ~, PName_Legs] = parroblib_load_robot(Structures{k}.Name, 0);
    RobNames = [RobNames, PName_Legs]; %#ok<AGROW>
  else
    RobNames = [RobNames, Structures{k}.Name]; %#ok<AGROW>
  end
end
RobNames = unique(RobNames);
% Prüfe, ob der Index für diese Optimierung vollständig ist
index_complete = false;
if ~make_global_index && exist(filename_idx, 'file')
  idx_data = load(filename_idx);
  if isfield(idx_data, 'RobNames')
    initpop_RobNames = idx_data.RobNames;
  else % altes Format ohne Speicherung der Roboternamen
    initpop_RobNames = {};
  end
  if length(intersect(RobNames,initpop_RobNames)) == length(RobNames)
    index_complete = true;
  end
end

if make_global_index
  % Wähle die Dateien zu allen Robotern aus. Es muss kein Wildcard (*) mehr
  % gesetzt werden, da die Ausdrücke unten bereits den * nach RobName haben
  RobNames = {''};
elseif index_complete % nur, falls nicht globaler Index genommen wird
  % Datei existiert schon und muss nur noch geladen werden. Nichts tun.
  % Dieser Fall tritt nur beim Debuggen auf, wenn mehrmals die gleiche
  % Optimierung gestartet wird.
  cds_log(2, sprintf('[gen_init_pop_index] Nutze bestehende Datei %s', filename_idx));
  return
else
  % Erzeuge die Datei nur für die gegebenen Roboter (Variable RobNames von oben)
end
cds_log(2, sprintf('[gen_init_pop_index] Beginne Zusammenfassung bisheriger Ergebnisse'));

initpop_matlist = {};
% Alle möglichen Ergebnis-Ordner durchgehen
for kk = 1:length(Set.optimization.result_dirs_for_init_pop)
  resdir = Set.optimization.result_dirs_for_init_pop{kk};
  % Unterordner sind die Ergebnis-Ordner einzelner Optimierungen oder
  % enthalten diese (Beide Verzeichnistiefen möglich)
  % Lade die Liste der Verzeichnisse mit Linux-Find-Befehl. Scheinbar ist
  % der Dateisystemzugriff auf dem Cluster über Matlab sehr langsam
  status = 1;
  if isunix()
    t2 = tic(); % Beginn der Prüfung auf Datei-Existenz
    t_ll = t2; % Zeitpunkt der letzten Log-Ausgabe diesbezüglich
    for j = 1:length(RobNames)
      RobName = RobNames{j};
      findcmd = sprintf(['find -L "%s" -maxdepth 3 ', ...
        '-name "Rob*_%s*_Endergebnis.mat"'], resdir, RobName);
      [status, matlist_j] = system(findcmd);
      % Erzeuge Liste der Mat-Dateien aus der Vorauswahl
      if status == 0
        matlist_j_cell = splitlines(matlist_j);
        I = ~strcmp(matlist_j_cell, '');
        initpop_matlist = [initpop_matlist;matlist_j_cell(I)]; %#ok<AGROW> 
      else
        cds_log(-1, sprintf(['Find-Befehl "%s" funktionierte nicht. ', ...
          'Status=%d Ausgabe:\n%s'], findcmd, status, matlist_j));
      end
      if toc(t_ll) > 20 || j == length(RobNames)
        cds_log(2, sprintf(['[gen_init_pop_index] Ergebnisse für Find-Dateisuche %d/%d', ...
          ' zusammengefasst. Dauer bis hier: %1.1fs'], j, length(RobNames), toc(t2)));
        t_ll = tic();
      end
    end
  end
  if status ~= 0 % Entweder Windows oder Find-Befehl erfolglos
    t2 = tic(); % Beginn dieser Prüfung auf Datei-Existenz
    t_ll = t2; % Zeitpunkt der letzten Log-Ausgabe diesbezüglich
    optdirs = dir(fullfile(resdir, '*'));
    % Füge Unter-Verzeichnisse hinzu (Rekursionstiefe 1)
    for i = 1:length(optdirs)
      if ~optdirs(i).isdir || optdirs(i).name(1) == '.'
        continue % Kein passendes Verzeichnis
      end
      if exist(fullfile(optdirs(i).folder, optdirs(i).name, ...
          [optdirs(i).name, '_settings.mat']), 'file')
        % Ordner ist selbst schon ein Ergebnisordner. Keine Rekursion.
        continue;
      end
      optdirs2 = dir(fullfile(resdir, optdirs(i).name, '*'));
      for j = 1:length(optdirs2)
        if ~optdirs2(j).isdir || optdirs2(j).name(1) == '.'
          continue % Kein passendes Verzeichnis
        end
        optdirs = [optdirs; optdirs2(j)]; %#ok<AGROW>
      end
    end
    for i = 1:length(optdirs) % Unterordner durchgehen.
      if ~optdirs(i).isdir || optdirs(i).name(1) == '.'
        continue % Kein passendes Verzeichnis
      end
      dirname_i = optdirs(i).name;
      % Aktuelle Roboter suchen
      for j = 1:length(RobNames)
        RobName = RobNames{j};
        resfiles = dir(fullfile(optdirs(i).folder, dirname_i, sprintf('Rob*_%s*_Endergebnis.mat',RobName)));
        III = find(contains({resfiles(:).name}, RobName));
        if toc(t_ll) > 20 || i == length(optdirs)
          cds_log(2, sprintf(['[gen_init_pop_index] Ergebnisse für Dir-Dateisuche %d/%d', ...
            ' zusammengefasst. Dauer bis hier: %1.1fs'], i, length(optdirs), toc(t2)));
          t_ll = tic();
        end
        if isempty(III)
          continue % Roboter nicht enthalten
        end
        for jj = III
          initpop_matlist = [initpop_matlist; ...
            fullfile(optdirs(i).folder, dirname_i, resfiles(jj).name )]; %#ok<AGROW> 
        end
      end
    end % for i ... optdirs
  end % if status
end
% Speichere Dateiliste ab
save(filename_idx, 'initpop_matlist', 'RobNames');
cds_log(2, sprintf(['[gen_init_pop_index] Vorherige Ergebnisse ', ...
  'zusammengefasst. Dauer: %1.1fs. Index-Datei: %s'], toc(t1), filename_idx));
