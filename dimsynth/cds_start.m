% Start der komb. Struktur und Maßsynthese für alle Roboter
% Erwartet Workspace-Variablen:
% Set (Globale Einstellungen)
% Traj (Eigenschaften der Trajektorie)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

% Warnungen unterdrücken, die bei der Maßsynthese typischerweise auftreten
warning('off', 'MATLAB:singularMatrix');
warning('off', 'MATLAB:nearlySingularMatrix');
warning('off', 'MATLAB:illConditionedMatrix');
warning('off', 'MATLAB:rankDeficientMatrix');
warning('off', 'Coder:MATLAB:singularMatrix');
warning('off', 'Coder:MATLAB:nearlySingularMatrix');
warning('off', 'Coder:MATLAB:illConditionedMatrix');
warning('off', 'Coder:MATLAB:rankDeficientMatrix');

%% Eingabe prüfen
if ~exist('Set', 'var') || ~exist('Traj', 'var')
  error('Eingabevariablen des Startskriptes existieren nicht');
end
fprintf('Starte Maßsynthese %s\n', Set.optimization.optname);
Set_default = cds_settings_defaults(struct('DoF', Set.structures.DoF));
for subconf = fields(Set_default)'
  for ftmp = fields(Set.(subconf{1}))'
    if ~isfield(Set_default.(subconf{1}), ftmp{1})
      warning('Feld %s in der Eingabestruktur ist nicht vorgesehen', ftmp{1})
    end
  end
end
if isa(Set.optimization.objective, 'char')
  % Bei mehrkriterieller Optimierung mehrere Zielfunktionsnamen als cell.
  % Einheitliches Format für Ein- und Mehrkriteriell.
  Set.optimization.objective = {Set.optimization.objective};
end
if Set.task.profile == 0 && any(strcmp(Set.optimization.objective, 'energy'))
  error('Energieberechnung ohne Zeitverlauf der Trajektorie nicht sinnvoll');
end
% Prüfe Plausibilität von Abbruchbedingungen und Wahl mehrkriterieller Ziele
if length(Set.optimization.obj_limit_physval) == 1 && length(Set.optimization.objective) > 1 && ...
    Set.optimization.obj_limit_physval == 0 % nur, falls Null nicht bereits überschrieben wurde
  % Korrigiere auf Dimension der Fitness-Funktion. Skalare Grenze von
  % Null ist Standard (kein Fehler)
  Set.optimization.obj_limit_physval = repmat(Set.optimization.obj_limit_physval, ...
    length(Set.optimization.objective), 1);
end
% Jetzt muss die Anzahl der Zielfunktionen mit der Abbruchbedingung stimmen
if length(Set.optimization.objective) ~= length(Set.optimization.obj_limit_physval)
  error('%d Zielfunktionen gesetzt und %d Abbruchbedingungen in obj_limit_physval. Passt nicht.', ...
    length(Set.optimization.objective), length(Set.optimization.obj_limit_physval));
end
if length(Set.optimization.obj_limit) == 1 && length(Set.optimization.objective) > 1 && ...
    Set.optimization.obj_limit == 0 % nur, falls Null nicht bereits überschrieben wurde
  Set.optimization.obj_limit = repmat(Set.optimization.obj_limit, ...
    length(Set.optimization.objective), 1);
end
if length(Set.optimization.objective) ~= length(Set.optimization.obj_limit)
  error('%d Zielfunktionen gesetzt und %d Abbruchbedingungen in obj_limit. Passt nicht.', ...
    length(Set.optimization.objective), length(Set.optimization.obj_limit));
end
if size(Set.optimization.obj_limit_physval,2) > 1
  error('obj_limit_physval muss %d x 1 Vektor sein', length(Set.optimization.objective));
end
if size(Set.optimization.obj_limit,2) > 1
  error('obj_limit muss %d x 1 Vektor sein', length(Set.optimization.objective));
end

if size(Set.task.installspace.params,1) ~= length(Set.task.installspace.type)
  error('Set.task.installspace: Länge von Feldern "params" und "type" stimmt nicht überein');
end
if length(Set.task.installspace.links) ~= length(Set.task.installspace.type)
  error('Set.task.installspace: Länge von Feldern "links" und "type" stimmt nicht überein');
end
if ~isa(Set.task.installspace.links, 'cell')
  error('Set.task.installspace: Feld "links" muss cell Array sein');
end
if size(Set.task.obstacles.params,1) ~= length(Set.task.obstacles.type)
  error('Set.task.obstacles: Länge von Feldern params und type stimmt nicht überein');
end
% Prüfe das Namensformat von Robotern auf der Positiv-Liste
for i = 1:length(Set.structures.whitelist)
  Name_i = Set.structures.whitelist{i};
  expression_P = 'P[\d][RP]+[\d]+[V]?[\d]*G[\d]+P[\d]+A[\d]+';
  expression_S = 'S[\d][RP]+[\d]+[V]?[\d]*';
  match_P = regexp(Name_i,expression_P,'match');
  match_S = regexp(Name_i,expression_S,'match');
  if isempty(match_P) && isempty(match_S)
    error('Roboter %s auf Positiv-Liste entspricht nicht dem Namensformat', Name_i);
  end
end
%% Menge der Roboter laden
Structures = cds_gen_robot_list(Set);

if isempty(Structures)
  fprintf('Keine Strukturen entsprechen den Filterkriterien\n');
  if ~isempty(Set.structures.whitelist)
    fprintf('Es wurde eine Positiv-Liste übergeben, aber keine Strukturen entsprachen den Kriterien. Filter-Liste passt nicht\n');
  end
  return
end

%% Ergebnis-Speicherort vorbereiten
if ~Set.general.computing_cluster
  resdir_main = fullfile(Set.optimization.resdir, Set.optimization.optname);
  mkdirs(resdir_main); % Ergebnis-Ordner für diese Optimierung erstellen
  % Einstellungen dieser kombinierten Synthese speichern. Damit ist im
  % Nachhinein nachvollziehbar, welche Roboter eventuell fehlen. Bereits hier
  % oben machen. Dann passt die Variable Structures auch für den Fall, dass
  % die Maßsynthese im folgenden Schritt aufgeteilt wird.
  save(fullfile(resdir_main, sprintf('%s_settings.mat', Set.optimization.optname)), ...
    'Set', 'Traj', 'Structures');
end
% Verzeichnisse zum Laden alter Ergebnisse vorbereiten
if Set.optimization.InitPopRatioOldResults > 0
  Set.optimization.result_dirs_for_init_pop = ...
    [Set.optimization.result_dirs_for_init_pop, Set.optimization.resdir];
else
  Set.optimization.result_dirs_for_init_pop = {};
end
%% Berechnung auf PBS-Cluster vorbereiten und durchführen
if Set.general.computing_cluster
  % Bereite eine Einstellungs-Datei vor
  % Folgende Zeile scheitert auf dem Cluster, da Pfad dort nicht gesetzt.
  % Das ist so gewollt.
  if isempty(which('computingcluster_repo_path.m'))
    error('Datei computingcluster_repo_path.m existiert nicht. Muss manuell aus template-Datei erstellt werden.');
  end
  cluster_repo_path = computingcluster_repo_path();
  % Erzeuge Liste aller oben zur Optimierung gefundener Roboter
  Names = {};
  for k = 1:length(Structures)
    Names = [Names, Structures{k}.Name]; %#ok<AGROW>
  end
  % Erzeuge Positiv-Liste für Cluster aus bereits ausgelesener Roboter-DB
  % Teile diese Liste so auf, dass mehrere Cluster-Instanzen parallel
  % gestartet werden können.
  I1_Struct = 1:Set.general.cluster_maxrobotspernode:length(Structures);
  if length(I1_Struct) > 1
    fprintf(['Teile die Optimierung von %d Robotern auf %d parallele Cluster-', ...
      'Instanzen auf\n'], length(Structures), length(I1_Struct));
  else
    fprintf('Lade die Optimierung von %d Robotern auf das Cluster hoch\n', length(Structures));
  end
  for kk = 1:length(I1_Struct)
    I1_kk = I1_Struct(kk); % Anfangs-Index in allen Roboter-Namen
    if kk < length(I1_Struct) % Bestimme End-Index
      I2_kk = I1_Struct(kk+1)-1;
    else
      I2_kk = length(Structures);
    end
    if length(I1_Struct) == 1 % Die Rechnung wird nicht aufgeteilt. Lasse den Namen wie er ist
      suffix = '';
    else % Rechnung aufgeteilt auf die verschiedenen Roboter. Suffix für Part-Nummer.
      suffix = sprintf('_p%d',kk);
    end
    computation_name = sprintf('dimsynth_%s_%s%s', ...
      datestr(now,'yyyymmdd_HHMMSS'), Set.optimization.optname, suffix);
    jobdir = fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
      'dimsynth', 'cluster_jobs', computation_name);
    mkdirs(fullfile(jobdir, 'results')); % Unterordner notwendig für Cluster-Transfer-Toolbox
    targetfile = fullfile(jobdir, [computation_name,'.m']);
    Set_cluster = Set;
    Set_cluster.optimization.optname = [Set.optimization.optname, suffix]; % sonst wird bei Zerlegung mehrfach der gleiche Name benutzt.
    Set_cluster.general.noprogressfigure = true; % Fortschritts-Bild des PSO auf Cluster nicht notwendig
    Set_cluster.general.plot_details_in_fitness = 0; % Keine Debug-Bilder
    Set_cluster.general.plot_robot_in_fitness = 0;
    Set_cluster.general.save_robot_details_plot_fitness_file_extensions = {}; % Auch nichts zum Debuggen speichern
    Set_cluster.general.computing_cluster = false; % auf Cluster muss "lokal" gerechnet werden
    Set_cluster.general.isoncluster = true; % Damit einige Debug-Bilder reduziert werden können
    Set_cluster.general.parcomp_struct = true; % parallele Berechnung auf Cluster (sonst sinnlos)
    Set_cluster.general.parcomp_plot = true; % paralleles Plotten auf Cluster (ist dort gleichwertig und schneller)
    % Wähle nur einen Bereich aller möglicher Roboter aus für diesen Lauf.
    Set_cluster.structures.whitelist = Names(I1_kk:I2_kk);

    save(fullfile(jobdir, [computation_name,'.mat']), 'Set_cluster', 'Traj');
    % Matlab-Skript erzeugen
    clusterheaderfile=fullfile(jobdir,'..','..','dimsynth_cluster_header.m');
    if ~exist(clusterheaderfile, 'file')
      error('Datei %s existiert nicht. Muss manuell aus template-Datei erstellt werden.', clusterheaderfile);
    end
    copyfile(clusterheaderfile, targetfile);
    fid = fopen(targetfile, 'a');
    fprintf(fid, 'tmp=load(''%s'');\n', [computation_name,'.mat']);
    fprintf(fid, 'Set=tmp.Set_cluster;\nTraj=tmp.Traj;\n');
    % Ergebnis-Ordner neu setzen. Ansonsten ist der Pfad des Rechners
    % gesetzt, von dem der Job gestartet wird.
    fprintf(fid, ['Set.optimization.resdir=fullfile(fileparts(', ...
      'which(''structgeomsynth_path_init.m'')),''results'');\n']);
    fprintf(fid, 'cds_start;\n');
    % Schließen des ParPools auch in Datei hineinschreiben
    fprintf(fid, 'parpool_writelock(''lock'', 300, true);\n');
    fprintf(fid, 'delete(gcp(''nocreate''));\n');
    fprintf(fid, 'parpool_writelock(''free'', 0, true);\n');
    fclose(fid);
    % Schätze die Rechenzeit: Im Mittel 2s pro Parametersatz aufgeteilt auf
    % 12 parallele Kerne, 30min für Bilderstellung und 6h Reserve/Allgemeines
    comptime_est = (Set.optimization.NumIndividuals*(1+Set.optimization.MaxIter)* ...
      2+30*60)*length(Set_cluster.structures.whitelist)/12 + 6*3600;
    % Matlab-Skript auf Cluster starten.
    addpath(cluster_repo_path);
    jobStart(struct('name', computation_name, ...
                    'matFileName', [computation_name, '.m'], ...
                    'locUploadFolder', jobdir, ...
                    'time',comptime_est/3600)); % Angabe in h
    fprintf(['Berechnung von %d Robotern wurde auf Cluster hochgeladen. Ende. ', ...
      'Die Ergebnisse müssen nach Beendigung der Rechnung manuell heruntergeladen ', ...
      'werden.\n'], length(Set_cluster.structures.whitelist));
  end
  if length(I1_Struct) > 1
    fprintf('Insgesamt %d Optimierungen mit in Summe %d Robotern hochgeladen\n',...
      length(I1_Struct), length(Structures));
  end
  return;
end
%% Vorbereitung und Durchführung der lokalen Optimierung
% Bei paralleler Berechnung dürfen keine Dateien geschrieben werden um
% Konflikte zu vermeiden
if Set.general.parcomp_struct && ... % Parallele Rechnung ist ausgewählt
    ~Set.general.regenerate_summmary_only && ... % für Bildgenerierung ParComp nicht benötigt
    length(Structures) > 1 % für Optimierung eines Roboters keine parallele Rechnung
  % Keine Bilder zeichnen
  Set.general.plot_details_in_fitness = 0;
  Set.general.plot_robot_in_fitness = 0;
  Set.general.noprogressfigure = true;
  % Keine (allgemeinen) mat-Dateien speichern
  Set.general.matfile_verbosity = 0;
  parpool_writelock('lock', 180, true); % Synchronisationsmittel für ParPool
  try
    parpool([1,Set.general.parcomp_maxworkers]);
  catch err
    fprintf('Fehler beim Starten des parpool: %s\n', err.message);
  end
  parpool_writelock('free', 0, true);
  Pool=gcp();
  parfor_numworkers = Pool.NumWorkers;
  if ~isinf(Set.general.parcomp_maxworkers) && parfor_numworkers ~= Set.general.parcomp_maxworkers
    warning('Die gewünschte Zahl von %d Parallelinstanzen konnte nicht erfüllt werden. Es sind jetzt %d.', ...
      Set.general.parcomp_maxworkers, parfor_numworkers)
  end
  % Warnungen auch in ParPool-Workern unterdrücken
  parfevalOnAll(gcp(), @warning, 0, 'off', 'MATLAB:singularMatrix');
  parfevalOnAll(gcp(), @warning, 0, 'off', 'MATLAB:nearlySingularMatrix');
  parfevalOnAll(gcp(), @warning, 0, 'off', 'MATLAB:illConditionedMatrix');
  parfevalOnAll(gcp(), @warning, 0, 'off', 'Coder:MATLAB:rankDeficientMatrix');
  parfevalOnAll(gcp(), @warning, 0, 'off', 'Coder:MATLAB:nearlySingularMatrix');
  parfevalOnAll(gcp(), @warning, 0, 'off', 'Coder:MATLAB:illConditionedMatrix');
else
  parfor_numworkers = 0;
end

if ~isempty(Set.structures.whitelist)
  if length(Set.structures.whitelist) ~= length(unique(Set.structures.whitelist))
    error('Die Positiv-Liste enthält doppelte Einträge');
  end
  Names_in_Struct = {}; % Es können bei Struktursynthese Strukturen doppelt getestet werden
  for i = 1:length(Structures), Names_in_Struct{i} = Structures{i}.Name; end %#ok<SAGROW>
  if length(Set.structures.whitelist) ~= length(unique(Names_in_Struct))
    warning('Es wurde eine Positiv-Liste übergeben, aber nicht alle dieser Strukturen wurden gewählt.');
    disp('Gültige PKM:');
    disp(intersect(Set.structures.whitelist, Names_in_Struct));
  end
end

if Set.optimization.use_desopt ... 
    && ~any(strcmp(Set.optimization.objective, {'mass', 'energy', 'stiffness'})) ...
    && ~any(Set.optimization.constraint_obj([1 5]))
  Set.optimization.use_desopt = false;
  fprintf(['Entwurfsoptimierung wurde verlangt, aber keine dafür notwendigen ', ...
    'Zielfunktionen oder Nebenbedingungen definiert. Wurde wieder deaktiviert\n']);
end
% Optimierung der Strukturen durchführen
if ~Set.general.regenerate_summmary_only
  % Vorbereitung: Getrennt für serielle und parallele Roboter
  for type = [0 2] % seriell und PKM
    % Stelle vorher eine Liste von Namen zusammen, um doppelte zu finden.
    Names = {};
    for k = 1:length(Structures)
      if Structures{k}.Type == type
        Names = [Names, Structures{k}.Name]; %#ok<AGROW>
      end
    end
    if isempty(Names), continue; end
    % Duplikate löschen (treten z.B. auf, wenn verschiedene Werte für theta
    % in der Struktursynthese möglich sind)
    Names = unique(Names);
    % Vorlagen-Funktionen neu generieren (falls dort Änderungen gemacht
    % wurden). Die automatische Neugenerierung in der parfor-Schleife
    % funktioniert nicht aufgrund von Dateikonflikten, autom. Ordnerlöschung.
    if Set.general.create_template_functions
      fprintf('Erstelle kompilierbare Funktionsdateien aus Vorlagen für %d Roboter\n', length(Names));
      III = 1:length(Names); % Zufällige Reihenfolge, damit besser parallelisierbar (Cluster)
      III = III(randperm(length(III)));
      for i = III
        if type == 0 % Serieller Roboter
          serroblib_create_template_functions(Names(i), false, false);
        else % PKM
          % Sperrschutz für PKM-Bibliothek (hauptsächlich für Struktursynthese)
          parroblib_writelock('check', 'csv', logical(Set.structures.DoF), 5*60, false);
          parroblib_create_template_functions(Names(i), false, false);
        end
        continue
      end
    end
    if Set.general.use_mex && Set.general.compile_missing_functions
      % Benötigte Funktionen kompilieren (serielle statt parallele Ausführung)
      % (es wird automatisch der codegen-Ordner gelöscht. Kann bei paralleler
      % Rechnung zu Konflikten führen)
      t1 = tic(); % Beginn der Prüfung auf Datei-Existenz
      t_ll = t1; % Zeitpunkt der letzten Log-Ausgabe diesbezüglich
      % Zufällige Reihenfolge, damit besser parallelisierbar (Cluster). Be-
      % trifft unabhängige Parallelinstanzen von Matlab.
      % Voraussetzung: rng('shuffle') vor Aufruf dieses Skripts
      III = 1:length(Names); 
      III = III(randperm(length(III)));
      for i = III
        if type == 0 % Serieller Roboter
          R = serroblib_create_robot_class(Names{i});
        else % PKM
          parroblib_writelock('check', 'csv', logical(Set.structures.DoF), 5*60, false);
          R = parroblib_create_robot_class(Names{i},1,1);
        end
        % Hierdurch werden fehlende mex-Funktionen kompiliert.
        if type == 2 % keine gleichzeitige mex-Kompilierung gleicher Kinematiken erlauben.
          parroblib_writelock('lock', Names{i}, logical(Set.structures.DoF), 60*60, Set.general.verbosity>2);
        end
        R.fill_fcn_handles(true, true);
        if type == 2 % Sperrschutz für PKM aufheben
          parroblib_writelock('free', Names{i}, logical(Set.structures.DoF));
        end
        if toc(t_ll) > 20 || i == III(end)
          fprintf('%d/%d Roboter vom Typ %d auf Existenz der Dateien geprüft. Dauer bis hier: %1.1fs\n', ...
            find(III==i,1,'first'), length(Names), type, toc(t1));
          t_ll = tic();
        end
      end
    end
  end
  t1 = tic();
  parfor (i = 1:length(Structures), parfor_numworkers)
    % Maßsynthese für diesen Roboter starten
    fprintf('Starte Maßsynthese für Roboter %d (%s)\n', i, Structures{i}.Name);
    cds_dimsynth_robot(Set, Traj, Structures{i});
  end
  fprintf('Optimierung von %d Robotern abgeschlossen. Dauer: %1.1fs\n', length(Structures), toc(t1));
end
if isempty(Structures)
  % Aufgrund der Filterkriterien wurden keine Roboter verglichen.
  % Auswertung ist nicht sinnvoll.
  return
end

%% Ergebnisse darstellen
cds_results_table(Set, Traj, Structures)
cds_vis_results(Set, Traj, Structures);
cds_create_evolution_videos(Set, Traj, Structures)
