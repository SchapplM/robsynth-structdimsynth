% Start der komb. Struktur und Maßsynthese für alle Roboter
% 
% Eingabe:
%   Set (Globale Einstellungen). Siehe cds_settings_defaults.m
%   Traj (Eigenschaften der Trajektorie). Siehe cds_gen_traj.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function cds_start(Set, Traj)

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
if ~Set.general.only_finish_aborted
  fprintf('Starte Maßsynthese %s.\n', Set.optimization.optname);
else
  fprintf('Schließe die abgebrochene Maßsynthese %s ab.\n', Set.optimization.optname);
end
if Set.general.only_finish_aborted && Set.general.regenerate_summary_only
  error('Option only_finish_aborted zusammen mit regenerate_summary_only nicht sinnvoll');
end
Set_default = cds_settings_defaults(struct('DoF', Set.task.DoF));
for subconf = fields(Set_default)'
  for ftmp = fields(Set.(subconf{1}))'
    if ~isfield(Set_default.(subconf{1}), ftmp{1})
      warning('Feld %s in der Eingabestruktur ist nicht vorgesehen', ftmp{1})
    end
  end
end
for ftmp = fields(Set)'
  if ~isfield(Set_default, ftmp{1})
    warning('Feld %s in der Eingabestruktur ist nicht vorgesehen', ftmp{1})
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
if Set.optimization.nolinkmass && any(strcmp(Set.optimization.objective, 'stiffness'))
  error('Berechnung der Steifigkeit zusammen mit nolinkmass aktuell nicht möglich');
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
if ~all(size(Set.optimization.base_size_limits)==[1 2])
  error('base_size_limits muss 1x2 sein min/max Radius');
end
if ~all(size(Set.optimization.platform_size_limits)==[1 2])
  error('platform_size_limits muss 1x2 sein min/max Radius');
end
if ~all(size(Set.optimization.basepos_limits)==[3 2])
  error('basepos_limits muss 3x2 sein (xyz Koordinate, min/max)');
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
if size(Set.task.installspace.type,2) > 1
  error('Set.task.installspace: Feld "type" hat mehr als eine Spalte.');
end
if size(Set.task.obstacles.params,1) ~= length(Set.task.obstacles.type)
  error('Set.task.obstacles: Länge von Feldern params und type stimmt nicht überein');
end
if size(Set.task.obstacles.type,2) > 1
  error('Set.task.obstacles: Feld "type" hat mehr als eine Spalte.');
end
assert(isa(Set.optimization.collshape_base, 'cell'), ...
  'Set.optimization.collshape_base muss cell array sein');
if length(intersect(Set.optimization.collshape_base, {'default', 'ring', ...
    'star', 'joint'})) ~= length(Set.optimization.collshape_base)
  error('Set.optimization.collshape_base enthält unerwarteten Wert');
end
if length(union(Set.optimization.desopt_vars, {'joint_stiffness_qref', ...
    'linkstrength'})) ~= 2
  error('Unerwarteter Wert in Set.optimization.desopt_vars');
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
if ~isempty(Set.structures.repeatlist)
  for j = 1:length(Set.structures.repeatlist)
    assert(isa(Set.structures.repeatlist{j}, 'cell'), ...
      'Eintrag in Set.structures.repeatlist ist kein Cell-Array');
    assert(length(Set.structures.repeatlist{j}) == 2, ...
      'Eintrag in Set.structures.repeatlist hat nicht Dimension 2');
  end
end
% Bei PKM-Struktursynthese darf nicht frühzeitig bei Singularität abbrechen
if length(Set.optimization.objective) == 1 && ... % ist immer einkriteriell
    any(strcmp(Set.optimization.objective, {'valid_act', 'valid_kin'}))
  Set.optimization.condition_limit_sing = inf; % Überschreibe Standardwert
end
assert(isa(Set.task.DoF, 'logical') && all(size(Set.task.DoF)==[1 6]), ...
  'Set.task.DoF muss 1x6 logical sein');
if ~isempty(Traj.X) && any(abs(Traj.X(1,:)-Traj.XE(1,:))>1e-6)
  error(['Erster Eckpunkt Traj.XE sollte identisch zum Trajektorienstart ', ...
    'Traj.X sein: [%s] vs [%s]'], disp_array(Traj.X(1,:), '%1.1f'), ...
    disp_array(Traj.XE(1,:), '%1.1f'));
end
if Set.general.save_evolution_video
  % Für dieses Bild müssen Bilder des Roboters im Verlauf der Optimierung
  % gespeichert werden
  Set.general.save_robot_details_plot_fitness_file_extensions = unique( ...
    [Set.general.save_robot_details_plot_fitness_file_extensions(:)', {'fig'}]);
  if Set.general.plot_robot_in_fitness < 1e3
    Set.general.plot_robot_in_fitness = 1e3;
  end
end
if isnan(Set.general.cluster_maxrobotspernode)
  Set.general.cluster_maxrobotspernode = Set.general.computing_cluster_cores;
end
%% Menge der Roboter laden
if ~(Set.general.only_finish_aborted && Set.general.isoncluster) && ... % Abschluss auf Cluster
    ~Set.general.regenerate_summary_only || ... % Nur Bilder (ohne Abschluss)
    (Set.general.only_finish_aborted && Set.general.computing_cluster) % Abschluss lokal. Kein Aufruf zum Hochladen des Abschluss-Auftrags auf das Cluster
  % Bei Fortsetzen der abgebrochenen Berechnung auf dem Cluster nicht
  % notwendig. Sonst schon (lokal oder Hochladen des Abschluss-Jobs).
  % Auch nicht notwendig bei reiner Neu-Erzeugung der Ergebnis-Bilder.
  % Der Fall des Hochladens des Auftrags zum Abschluss auf das Cluster wird
  % mit obiger Logik berücksichtigt (zum Aufteilen auf parallele Jobs)
  Structures = cds_gen_robot_list(Set);
  if isempty(Structures)
    fprintf('Keine Strukturen entsprechen den Filterkriterien\n');
    if ~isempty(Set.structures.whitelist)
      fprintf('Es wurde eine Positiv-Liste übergeben, aber keine Strukturen entsprachen den Kriterien. Filter-Liste passt nicht\n');
    end
    return
  end
end
%% Ergebnis-Speicherort vorbereiten. Einstellungen speichern.
resdir_main = fullfile(Set.optimization.resdir, Set.optimization.optname);
settingsfile = fullfile(resdir_main, sprintf('%s_settings.mat', ...
  Set.optimization.optname));
if Set.general.only_finish_aborted && (Set.general.isoncluster || ...
    ~Set.general.computing_cluster) % nicht bei hochladen des Jobs aufs Cluster
  % Alte Einstellungsdatei laden. Damit muss nicht die Menge der Strukturen
  % neu erzeugt werden (geht schneller und ist robuster).
  if ~exist(resdir_main, 'file')
    fprintf(['Nachträglicher Abschluss von %s nicht möglich. Verzeichnis %s ', ...
      'fehlt.\n'], Set.optimization.optname, resdir_main);
    return
  end
  if ~exist(settingsfile, 'file')
    fprintf(['Nachträglicher Abschluss von %s nicht möglich. Datei %s ', ...
      'fehlt.\n'], Set.optimization.optname, settingsfile);
    return
  end
  d = load(settingsfile, 'Set', 'Traj', 'Structures');
  Traj = d.Traj;
  Structures = d.Structures;
  Set_tmp = Set;
  Set = cds_settings_update(d.Set);
  % Überschreibe geladene Einstellungen, damit ein kopierter Ordner vom
  % Cluster auch lokal abgeschlossen werden kann.
  Set.general.only_finish_aborted = true; % Überschreibe geladene Einstellung
  Set.general.parcomp_plot = Set_tmp.general.parcomp_plot;
  Set.general.parcomp_struct = Set_tmp.general.parcomp_struct;
  Set.optimization.resdir = Set_tmp.optimization.resdir;
  Set.general.create_template_functions = Set_tmp.general.create_template_functions;
  fprintf('Einstellungsdatei %s für Abschluss geladen.\n', settingsfile);
  % Prüfe, ob der Abschluss noch notwendig ist. Annahme: Ist die Ergebnis-
  % Tabelle einmal erstellt, sind alle einzelnen Roboter abgeschlossen.
  restabfile = fullfile(resdir_main, sprintf('%s_results_table.csv', ...
    Set.optimization.optname));
  if exist(restabfile, 'file')
    fprintf(['Ergebnis-Tabelle existiert schon. Kein Abschluss der abge', ...
      'brochenen Berechnung notwendig.\n']);
    return
  end
elseif Set.general.regenerate_summary_only && (Set.general.isoncluster || ...
    ~Set.general.computing_cluster)
  % Es sollen nur die Bilder neu generiert werden. Lade die alten Ein-
  % stellungen, damit die Nummern der Roboter nicht geändert werden.
  d = load(settingsfile, 'Set', 'Traj', 'Structures');
  Traj = d.Traj;
  % Übernehme Optionen zur Steuerung der Bildgenerierung aus der Eingabe
  Set_tmp = Set;
  Set = d.Set;
  Set.general.eval_figures = Set_tmp.general.eval_figures;
  Set.general.animation_styles = Set_tmp.general.animation_styles;
  Set.general.parcomp_plot = Set_tmp.general.parcomp_plot;
  Set.general.parcomp_struct = false; % keine Struktursynthese.
  Set.general.regenerate_summary_only = true;
  Set.general.nosummary = Set_tmp.general.nosummary; % für nur Tabelle ohne Bilder
  Set.optimization.resdir = Set_tmp.optimization.resdir; % anders auf Cluster
  Structures = d.Structures;
  fprintf('Einstellungsdatei %s für Bild-Generierung geladen.\n', settingsfile);
elseif ~Set.general.computing_cluster % nicht bei Hochladen des Jobs
  mkdirs(resdir_main); % Ergebnis-Ordner für diese Optimierung erstellen
  % Einstellungen dieser kombinierten Synthese speichern. Damit ist im
  % Nachhinein nachvollziehbar, welche Roboter eventuell fehlen. Bereits hier
  % oben machen. Dann passt die Variable Structures auch für den Fall, dass
  % die Maßsynthese im folgenden Schritt aufgeteilt wird.
  save(settingsfile, 'Set', 'Traj', 'Structures');
end
% Verzeichnisse zum Laden alter Ergebnisse vorbereiten
if Set.optimization.InitPopRatioOldResults > 0
  % Bei Hochladen der Berechnung auf das Cluster darf nicht das lokale
  % Verzeichnis eingetragen werden. Sonst immer das lokale Verzeichnis.
  if ~(Set.general.computing_cluster && ~Set.general.isoncluster)
    Set.optimization.result_dirs_for_init_pop = unique( ... % bei mehrfacher Durchführung von cds_start sonst doppelt
      [Set.optimization.result_dirs_for_init_pop, Set.optimization.resdir]);
  end
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
  jobIDs = NaN(2,length(I1_Struct)); % erste Zeile: Produktiv-Job; zweite Zeile: Aufräum-Job
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
    if Set.general.only_finish_aborted
      % Es wird keine Optimierung durchgeführt. Kennzeichnung im Namen.
      computation_name = [computation_name, '_finish']; %#ok<AGROW>
    end
    jobdir = fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
      'dimsynth', 'cluster_jobs', computation_name);
    mkdirs(fullfile(jobdir, 'results')); % Unterordner notwendig für Cluster-Transfer-Toolbox
    targetfile = fullfile(jobdir, [computation_name,'.m']);
    Set_cluster = Set;
    Set_cluster.optimization.optname = [Set.optimization.optname, suffix]; % sonst wird bei Zerlegung mehrfach der gleiche Name benutzt.
    Set_cluster.general.noprogressfigure = true; % Fortschritts-Bild des PSO auf Cluster nicht notwendig
    Set_cluster.general.computing_cluster = false; % auf Cluster muss "lokal" gerechnet werden
    Set_cluster.general.isoncluster = true; % Damit einige Debug-Bilder reduziert werden können
    Set_cluster.general.parcomp_struct = true; % parallele Berechnung auf Cluster (sonst sinnlos)
    Set_cluster.general.parcomp_plot = true; % paralleles Plotten auf Cluster (ist dort gleichwertig und schneller)
    % Wähle nur einen Bereich aller möglicher Roboter aus für diesen Lauf.
    Set_cluster.structures.whitelist = Names(I1_kk:I2_kk);
    % Falls ein Roboter mehrfach parallel optimiert werden soll, muss die
    % Einstellungen für das Cluster neu generiert werden.
    if ~isempty(Set.structures.repeatlist)
      Set_cluster.structures.repeatlist = {};
      Names_repeated = Set_cluster.structures.whitelist;
      Set_cluster.structures.whitelist = unique(Names_repeated);
      % Zähle, wie viele Roboter jeweils doppelt sind. Daraus wird die Ein-
      % stellung repeatlist rekonstruiert.
      for jj = 1:length(Set_cluster.structures.whitelist)
        I_unique = strcmp(Names_repeated, Set_cluster.structures.whitelist{jj});
        if sum(I_unique) == 1, continue; end % Nur eine Optimierung. Kein Eintrag notwendig.
        Set_cluster.structures.repeatlist = [Set_cluster.structures.repeatlist, ...
          {{Set_cluster.structures.whitelist{jj}, sum(I_unique)}}];
      end
    end

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
    % Platzhalter-Eintrag, der weiter unten ersetzt wird:
    fprintf(fid, '%% Set.general.only_finish_aborted = true;\n');
    fprintf(fid, 'cds_start(Set,Traj);\n');
    % Schließen des ParPools auch in Datei hineinschreiben
    fprintf(fid, 'parpool_writelock(''lock'', 300, true);\n');
    fprintf(fid, 'delete(gcp(''nocreate''));\n');
    fprintf(fid, 'parpool_writelock(''free'', 0, true);\n');
    fclose(fid);
    % Schätze die Rechenzeit: Im Mittel 2s pro Parametersatz aufgeteilt auf
    % 12 parallele Kerne, 30min für Bilderstellung und 6h Reserve/Allgemeines
    comptime_est = (Set.optimization.NumIndividuals*(1+Set.optimization.MaxIter)*2 + ...
      30*60)*ceil(length(I1_kk:I2_kk)/12) + 6*3600;
    % Falls Entwurfsoptimierung durchgeführt wird, rechne dort auch noch
    % mit 1s pro Partikel, durchschnittlich 20 Iterationen bei 10% aller
    % Partikel aus der Maßsynthese.
    if ~isempty(Set.optimization.desopt_vars)
      npart = Set.optimization.NumIndividuals*(1+Set.optimization.MaxIter);
      comptime_est = comptime_est + 0.1*npart*1*20;
    end
    if Set.general.only_finish_aborted || Set.general.regenerate_summary_only
      % Es wird keine Optimierung durchgeführt. Überschreibe die vorher
      % berechnete Zeit (nur Bilderstellung).
      comptime_est = ceil(length(I1_kk:I2_kk)/12)*30*60;
    end
    if ~isnan(Set.general.computing_cluster_max_time)
      if length(I1_kk:I2_kk) > Set.general.computing_cluster_cores
        warning(['Keine Nutzung von computing_cluster_max_time möglich ', ...
          '(Jobs nicht voll parallel)']);
      else
        comptime_est = Set.general.computing_cluster_max_time;
      end
    end
    % Matlab-Skript auf Cluster starten.
    addpath(cluster_repo_path);
    ppn = min(length(I1_kk:I2_kk),Set.general.computing_cluster_cores);
    dependstruct = struct('');
    if ~isempty(Set.general.cluster_dependjobs)
      dependstruct = struct('afterok', Set.general.cluster_dependjobs);
    end
    jobIDs(1,kk) = jobStart(struct( ...
      'name', computation_name, ...
      ... % Nur so viele Kerne beantragen, wie auch benötigt werden ("ppn")
      'ppn', ppn, ... % 32 ist max. auf Cluster
      'mem', 24+4*ppn, ... % mit 30GB kam es öfter mal zu Speicherfehlern
      'matFileName', [computation_name, '.m'], ...
      'locUploadFolder', jobdir, ...
      'time',comptime_est/3600), ... % Angabe in h
      dependstruct); % Mögliche Abhängigkeiten (optional)
    % Zusätzlich direkt das Aufräum-Skript starten. Es ist davon auszugehen, 
    % dass der Job vorzeitig abgebrochen wird, da die Rechenzeit unterschätzt
    % wird.
    if Set.general.only_finish_aborted
      continue % In diesem Fall war der Zweck des Aufrufs schon das Aufräum-Skript
    end
    % Neues Skript vorbereiten (im gleichen Ordner)
    computation_name2 = [computation_name,'_finish'];
    targetfile2 = fullfile(jobdir, [computation_name2,'.m']);
    copyfile(targetfile,targetfile2);
    f = strrep(fileread(targetfile2), '% Set.general.only_finish_aborted', ...
      'Set.general.only_finish_aborted');
    fid  = fopen(targetfile2,'w'); fprintf(fid,'%s',f); fclose(fid);
    % Skript hochladen mit dem vorherigen Job als Abhängigkeit. Wenn der
    % Job vom PBS abgebrochen wird, wird der Aufräum-Job gestartet. Bei
    % Erfolg verfällt der Aufräum-Job.
    jobIDs(2,kk) = jobStart(struct( ...
      'name', computation_name2, ...
      'ppn', min(length(I1_kk:I2_kk),Set.general.computing_cluster_cores), ... % gleiche Anzahl wie oben
      'matFileName', [computation_name2, '.m'], ...
      'locUploadFolder', jobdir, ...
      'time',2), ... % % Geht schnell. Veranschlage 2h. Evtl. länger wegen ParPool-Synchronisation.
      struct('afternotok', jobIDs(1,kk))); % Abbruch des vorherigen Jobs als Start-Bedingung
    fprintf(['Berechnung von %d Robotern wurde auf Cluster hochgeladen. Ende. ', ...
      'Die Ergebnisse müssen nach Beendigung der Rechnung manuell heruntergeladen ', ...
      'werden.\n'], length(I1_kk:I2_kk));
    if kk < length(I1_Struct) % Damit nicht alle exakt zeitgleich starten; exakt gleichzeitiger, ...
      pause(30); % ... paralleler Start des parpools sowieso nicht möglich
    end
  end
  if length(I1_Struct) > 1 && ~Set.general.only_finish_aborted
    % Bei mehreren aufgeteilten Läufen direkt die Zusammenfassung aller Teile
    % als zusätzlichen Job starten
    computation_name3 = sprintf('dimsynth_%s_%s%s', ...
      datestr(now,'yyyymmdd_HHMMSS'), Set.optimization.optname, '_merge');
    jobdir3 = fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
      'dimsynth', 'cluster_jobs', computation_name3);
    mkdirs(fullfile(jobdir3, 'results')); % Unterordner notwendig für Cluster-Transfer-Toolbox
    targetfile3 = fullfile(jobdir3, [computation_name3,'.m']);
    clusterheaderfile=fullfile(jobdir3,'..','..','dimsynth_cluster_header.m');
    if ~exist(clusterheaderfile, 'file')
      error('Datei %s existiert nicht. Muss manuell aus template-Datei erstellt werden.', clusterheaderfile);
    end
    copyfile(clusterheaderfile, targetfile3);
    fid = fopen(targetfile3, 'a');
    fprintf(fid, 'cds_merge_results( ''%s'', ''move'', true, true );', ...
      Set.optimization.optname);
    fclose(fid);
    % Zusammenfassungs-Job auf Cluster hochladen
    jobStart(struct( ...
      'name', computation_name3, ...
      'ppn', 1, ... % gleiche Anzahl wie oben
      'matFileName', [computation_name3, '.m'], ...
      'locUploadFolder', jobdir3, ...
      'time',1), ... % Geht schnell
      ... % Nur starten, wenn vorherige Produktiv- und Aufräum-Jobs erledigt
      struct('afterany', jobIDs(:)'));
    fprintf('Insgesamt %d Optimierungen mit in Summe %d Robotern hochgeladen\n',...
      length(I1_Struct), length(Structures));
  end
  return;
end
%% Vorbereitung und Durchführung der lokalen Optimierung
% Bei paralleler Berechnung dürfen keine Dateien geschrieben werden um
% Konflikte zu vermeiden
if Set.general.parcomp_struct && ... % Parallele Rechnung ist ausgewählt
    ~Set.general.regenerate_summary_only && ... % für Bildgenerierung ParComp nicht benötigt
    length(Structures) > 1 % für Optimierung eines Roboters keine parallele Rechnung
  Set.general.noprogressfigure = true;
  % Keine (allgemeinen) mat-Dateien speichern
  Set.general.matfile_verbosity = 0;
  if Set.general.isoncluster % auf Cluster möglicher Zugriffskonflikt für ParPool
    parpool_writelock('lock', 180, true); % Synchronisationsmittel für ParPool
  end
  Pool = gcp('nocreate');
  if isempty(Pool)
    try
      Pool=parpool([1,Set.general.parcomp_maxworkers]);
      parfor_numworkers = Pool.NumWorkers;
    catch err
      fprintf('Fehler beim Starten des parpool: %s\n', err.message);
      parfor_numworkers = 1;
    end
  else
    parfor_numworkers = Pool.NumWorkers;
  end
  clear Pool
  if Set.general.isoncluster
    parpool_writelock('free', 0, true);
  end
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
  % Siehe https://github.com/altmany/export_fig/issues/75
  parfevalOnAll(gcp(), @warning, 0, 'off', 'MATLAB:prnRenderer:opengl');
else
  parfor_numworkers = 0;
end

if ~isempty(Set.structures.whitelist)
  if length(Set.structures.whitelist) ~= length(unique(Set.structures.whitelist))
    error('Die Positiv-Liste enthält doppelte Einträge');
  end
  Names_in_Struct = {}; % Es können bei Struktursynthese Strukturen doppelt getestet werden
  for i = 1:length(Structures), Names_in_Struct{i} = Structures{i}.Name; end %#ok<AGROW>
  if length(Set.structures.whitelist) ~= length(unique(Names_in_Struct))
    warning('Es wurde eine Positiv-Liste übergeben, aber nicht alle dieser Strukturen wurden gewählt.');
    disp('Gültige Roboter:');
    disp(intersect(Set.structures.whitelist, Names_in_Struct));
  end
end

if ~isempty(Set.optimization.desopt_vars)
  valid_desopt = false;
  % Zielfunktion oder Nebenbedingung basierend auf Dynamik und diese
  % beeinflussende Entwurfsoptimierung
  if any(strcmp(Set.optimization.desopt_vars, 'linkstrength')) && ...
      (~isempty(intersect(Set.optimization.objective, {'mass', 'energy', ...
      'stiffness', 'materialstress'})) || any(Set.optimization.constraint_obj([1 2 3 5 6])))
    valid_desopt = true;
  end
  if any(strcmp(Set.optimization.desopt_vars, 'joint_stiffness_qref')) && ...
      Set.optimization.joint_stiffness_passive_revolute && ...
      (~isempty(intersect(Set.optimization.objective, {'mass', 'energy', ...
      'materialstress'})) || any(Set.optimization.constraint_obj([1 2 3 5 6])))
    valid_desopt = true;
  end
  if ~valid_desopt
    Set.optimization.desopt_vars = {};
    fprintf(['Entwurfsoptimierung wurde verlangt, aber keine dafür notwendigen ', ...
      'Zielfunktionen oder Nebenbedingungen definiert. Wurde wieder deaktiviert\n']);
  end
end
% Optimierung der Strukturen durchführen
if ~Set.general.regenerate_summary_only
  % Vorbereitung: Getrennt für serielle und parallele Roboter
  for type = [0 2] % seriell und PKM
    % Stelle vorher eine Liste von Namen zusammen, um doppelte zu finden.
    Names = {};
    I_type = false(length(Structures), 1);
    for k = 1:length(Structures)
      if Structures{k}.Type == type
        Names = [Names, Structures{k}.Name]; %#ok<AGROW>
        I_type(k) = true;
      end
    end
    if isempty(Names), continue; end
    Structures_Type = Structures(I_type); % Strukturen des gesuchten Typs
    % Duplikate löschen (treten z.B. auf, wenn verschiedene Werte für theta
    % in der Struktursynthese möglich sind)
    [Names, I] = unique(Names);
    Structures_I = Structures_Type(I);
    % Entferne doppelte PKM-Namen, die sich nur durch die Gestell-/Platt-
    % formausrichtung unterscheiden. Die Dateien sind gleich.
    if type == 2
      PNames_Legs = cell(size(Names)); % PKM-Namen bezogen auf Beinketten-Kinematik (ohne G-/P-Nummer)
      SNames_Legs = cell(size(Names)); % Namen der Beinketten
      for i = 1:length(Names) % Bestimme Namen der PKM ohne P-/G-Nummer
        [~,Leg_Names_i,~,~,~,~,~,~,PNames_Legs{i}] = parroblib_load_robot(Names{i}, 0);
        SNames_Legs{i} = Leg_Names_i{1}; % Annahme: symmetrische PKM
      end
      [~,I] = unique(PNames_Legs);
      Names = Names(I);
      SNames_Legs = unique(SNames_Legs);
      Structures_I = Structures_Type(I);
    end
    % Vorlagen-Funktionen neu generieren (falls dort Änderungen gemacht
    % wurden). Die automatische Neugenerierung in der parfor-Schleife
    % funktioniert nicht aufgrund von Dateikonflikten, autom. Ordnerlöschung.
    if Set.general.create_template_functions
      fprintf('Erstelle kompilierbare Funktionsdateien aus Vorlagen für %d Roboter\n', length(Names));
      tplmode = false; % Erzeuge alle Dateien neu (auch wenn schon vorhanden)
    else
      tplmode = true; % Erzeuge nur fehlende Dateien neu (sonst später Fehler)
    end
    III = 1:length(Names); % Zufällige Reihenfolge, damit besser parallelisierbar (Cluster)
    III = III(randperm(length(III)));
    for i = III
      Structure_i = Structures_I{i};
      if type == 0 % Serieller Roboter
        serroblib_create_template_functions(Names(i), tplmode, false);
      else % PKM
        % Zuerst Funktionen für serielle Beinketten neu generieren.
        % Eine PKM-Funktion (Beinketten-IK) ist davon abhängig.
        [~, LEG_Names] = parroblib_load_robot(Names{i}, 0);
        serroblib_writelock('lock', 'template', 0, 5*60, false);
        serroblib_create_template_functions(LEG_Names(1), tplmode, false);
        serroblib_writelock('free', 'template', 0, 5*60, false);
        % Sperrschutz für PKM-Bibliothek (hauptsächlich für Struktursynthese)
        parroblib_writelock('check', 'csv', Structure_i.DoF, 5*60, false);
        % Die Vorlagen-Funktionen können nicht in Parallelinstanzen
        % gleichzeitig erzeugt werden.
        parroblib_writelock('lock', 'template', Structure_i.DoF, 5*60, false);
        parroblib_create_template_functions(Names(i), tplmode, false);
        parroblib_writelock('free', 'template', Structure_i.DoF, 5*60, false);
      end
    end
    if Set.general.update_template_functions
      if type == 0
        serroblib_writelock('lock', 'template', 0, 5*60, false);
        serroblib_update_template_functions(Names,Set.general.verbosity>2)
        serroblib_writelock('free', 'template', 0, 5*60, false);
      else
        % Zuerst die Vorlagen-Funktionen für die seriellen Beinketten
        serroblib_writelock('lock', 'template', 0, 5*60, false);
        serroblib_update_template_functions(SNames_Legs,Set.general.verbosity>2)
        serroblib_writelock('free', 'template', 0, 5*60, false);
        % Danach die Funktionen für die PKM
        parroblib_writelock('lock', 'template', Structures_I{1}.DoF, 5*60, false);
        parroblib_update_template_functions(Names,Set.general.verbosity>2)
        parroblib_writelock('free', 'template', Structures_I{1}.DoF, 5*60, false);
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
          parroblib_writelock('check', 'csv', logical(Set.task.DoF), 5*60, false);
          R = parroblib_create_robot_class(Names{i},1,1);
        end
        % Hierdurch werden fehlende mex-Funktionen kompiliert.
        if type == 2 % keine gleichzeitige mex-Kompilierung gleicher Kinematiken erlauben.
          parroblib_writelock('lock', Names{i}, R.I_EE, 60*60, Set.general.verbosity>2);
          serroblib_writelock('lock', R.Leg(1).mdlname, NaN, 60*60, Set.general.verbosity>2); % Auch SerRobLib für Beinkette sperren
        end
        R.fill_fcn_handles(true, true);
        if type == 2 % Sperrschutz für PKM und Beinkette aufheben
          serroblib_writelock('free', R.Leg(1).mdlname, NaN); 
          parroblib_writelock('free', Names{i}, R.I_EE);
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
    % Auflösung für Debug-Bilder setzen (wird auf ParPool auf Cluster nicht
    % vererbt aus globalen Einstellungen)
    if parfor_numworkers > 0
      set(0, 'defaultfigureposition', [1 1 1920 1080]);
      set(0, 'defaultfigureunits', 'pixels');
    end
    % Maßsynthese für diesen Roboter starten
    if ~Set.general.only_finish_aborted, mode = 'Maßsynthese'; %#ok<PFBNS>
    else,                                mode = 'Abschluss'; end
    if isempty(Structures{i}.RobName), RobNameStr = '';
    else, RobNameStr = sprintf('; %s', Structures{i}.RobName); end
    fprintf('Starte %s für Roboter %d (%s%s)\n', mode, i, Structures{i}.Name, RobNameStr);
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
cds_results_table(Set, Traj, Structures);
cds_vis_results(Set, Traj, Structures);
cds_create_evolution_videos(Set, Traj, Structures);
