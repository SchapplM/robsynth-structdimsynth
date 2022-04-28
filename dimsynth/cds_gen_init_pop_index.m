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
cds_log(2, sprintf('[gen_init_pop_index] Beginne Zusammenfassung bisheriger Ergebnisse'));
t1 = tic();
RobNames = {};
if ~make_global_index
  for k = 1:length(Structures)
    RobNames = [RobNames, Structures{k}.Name]; %#ok<AGROW>
  end
  RobNames = unique(RobNames);
else
  % Wähle die Dateien zu allen Robotern aus. Es muss kein Wildcard (*) mehr
  % gesetzt werden, da die Ausdrücke unten bereits den * nach RobName haben
  RobNames = {''};
end

initpop_matlist = {};
% Alle möglichen Ergebnis-Ordner durchgehen
for kk = 1:length(Set.optimization.result_dirs_for_init_pop)
  resdir = Set.optimization.result_dirs_for_init_pop{kk};
  % Unterordner sind die Ergebnis-Ordner einzelner Optimierungen
  % Lade die Liste der Verzeichnisse mit Linux-Find-Befehl. Scheinbar ist
  % der Dateisystemzugriff auf dem Cluster über Matlab sehr langsam
  status = 1;
  if isunix()
    for j = 1:length(RobNames)
      RobName = RobNames{j};
      [status,matlist_j] = system(sprintf(['find -L "%s" -maxdepth 2 ', ...
        '-name "Rob*_%s*_Endergebnis.mat"'], resdir, RobName));
      % Erzeuge Liste der Mat-Dateien aus der Vorauswahl
      if status == 0
        matlist_j_cell = splitlines(matlist_j);
        I = ~strcmp(matlist_j_cell, '');
        initpop_matlist = [initpop_matlist;matlist_j_cell(I)]; %#ok<AGROW> 
      else
        cds_log(-1, sprintf('Find-Befehl funktionierte nicht in %s. Ausgabe:\n%s', resdir, dirlist));
      end
    end
  end
  if status ~= 0 % Entweder Windows oder Find-Befehl erfolglos
    optdirs = dir(fullfile(resdir, '*'));
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
        if isempty(III)
          continue % Roboter nicht enthalten
        end
        for jj = III
          initpop_matlist = [initpop_matlist; ...
            fullfile(optdirs(i).folder, dirname_i, resfiles(III).name )]; %#ok<AGROW> 
        end
      end
    end
  end % if status
end
% Speichere Dateiliste ab
if make_global_index
  filename_idx = fullfile(Set.optimization.resdir, 'index_results.mat');
else
  resdir_main = fullfile(Set.optimization.resdir, Set.optimization.optname);
  mkdirs(fullfile(resdir_main, 'tmp'));
  filename_idx = fullfile(resdir_main, 'tmp', 'old_results.mat');
end
save(filename_idx, 'initpop_matlist');
cds_log(2, sprintf(['[gen_init_pop_index] Vorherige Ergebnisse ', ...
  'zusammengefasst. Dauer: %1.1fs. Index-Datei: %s'], toc(t1), filename_idx));