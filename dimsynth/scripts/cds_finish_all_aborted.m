% Stelle alle abgebrochenen Instanzen der Maßsynthese fertig

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
 % Schließe nur Optimierungen nochmal ab, die jünger als 7 Tage sind
max_age_optim_days = 7;

% Alle Optimierungsordner durchgehen
repopath = fileparts(which('structgeomsynth_path_init.m'));
respath = fullfile(repopath, 'results');
optdirs = dir(fullfile(respath, '*'));
fprintf(['Gehe durch alle %d Unterordner von %s und prüfe auf Abschluss ', ...
  'abgebrochener Optimierungen.\n'], length(optdirs), respath);
for i = 1:length(optdirs)
  if ~optdirs(i).isdir
    continue
  end
  if optdirs(i).name(1) == '.'
    continue
  end
  %% Prüfe, ob der Ordner der Optimierung zu alt ist.
  if now() - optdirs(i).datenum > max_age_optim_days
    fprintf('%s: Zu alt (%1.1f Tage)\n', optdirs(i).name, now() - optdirs(i).datenum);
    continue
  end
  % Prüfe, ob Optimierung schon abgeschlossen ist
  sf = fullfile(respath,optdirs(i).name,[optdirs(i).name,'_settings.mat']);
  if ~exist(sf, 'file')
    fprintf('%s: Einstellungsdatei existiert nicht\n', optdirs(i).name);
    continue % Altes Format oder ungültiges Verzeichnis
  end
  sd = load(sf);
  if ~isfield(sd, 'Structures')
    fprintf('%s: Einstellungsdatei hat kein Feld Structures\n', optdirs(i).name);
    continue % Altes Format oder ungültiges Verzeichnis
  end
  complete = true(length(sd.Structures),1);
  for j = 1:length(sd.Structures)
    if ~exist(fullfile(respath,optdirs(i).name, sprintf('Rob%d_%s_Endergebnis.mat', ...
        sd.Structures{j}.Number, sd.Structures{j}.Name)), 'file')
      complete(j) = false;
    end
  end
  if all(complete)
    fprintf('%s: Alle Optimierungen abgeschlossen.\n', optdirs(i).name);
    continue % Kein Abschluss notwendig
  end
  active = false(length(sd.Structures),1);
  %% Prüfe, ob Log-Dateien noch aktiv sind
  for j = 1:length(sd.Structures)
    logfile = fullfile(respath,optdirs(i).name, sprintf('Rob%d_%s', ...
        sd.Structures{j}.Number, sd.Structures{j}.Name), sprintf('Rob%d_%s.log', ...
        sd.Structures{j}.Number, sd.Structures{j}.Name));
    dl = dir(logfile);
    if isempty(dl), continue; end
    logage = now() - dl(1).datenum;
    if logage < 5/(24*60) % Jünger als 5min. Vermutlich noch aktiv
      active(j) = true;
    end
  end
  if any(active)
    fprintf('%s: Optimierung läuft noch\n', optdirs(i).name);
    continue % Abschluss noch nicht möglich. Optimierung läuft noch
  end
  %% Prüfe, ob die Temp-Dateien zum Abschluss überhaupt vorhanden sind
  tmpfiles_available = false(length(sd.Structures),1);
  for j = 1:length(sd.Structures)
    resdir_tmp = fullfile(respath,optdirs(i).name, 'tmp', ...
      sprintf('%d_%s', sd.Structures{j}.Number, sd.Structures{j}.Name));
    filelist_tmpres = dir(fullfile(resdir_tmp, '*_Gen*_AllInd.mat'));
    if ~isempty(filelist_tmpres)
      tmpfiles_available(j) = true;
    end
    % Suche nach weiteren tmp-Dateien (konvertiert von cds_gen_init_pop)
    filelist_tmpres2 = dir(fullfile(respath,optdirs(i).name, sprintf( ...
      'Rob%d_%s_Endergebnis_Gen*.mat', sd.Structures{j}.Number, sd.Structures{j}.Name)));
    if ~isempty(filelist_tmpres2)
      tmpfiles_available(j) = true;
    end
  end

  tf_file = fullfile(respath,optdirs(i).name,[optdirs(i).name,'_results_table.csv']);
  if any(complete) && ~exist(tf_file, 'file')
    fprintf('%s: Ergebnis-Tabelle existiert nicht: %s\n', optdirs(i).name, tf_file);
  elseif ~any(~complete & tmpfiles_available)
    % Es gibt keine Wiederherstellungsdateien, die helfen würden
    fprintf('%s: Keine Wiederherstellungsdateien\n', optdirs(i).name);
    continue
  end
  %% Schließe die Optimierung erneut ab
  Set_tmp = cds_settings_update(sd.Set, 1);
  Set_tmp.general.computing_cluster = false; % Abschluss muss immer lokal gemacht werden bezogen auf System, das dieses Skript hier ausführt
  Set_tmp.general.only_finish_aborted = true;
  Set_tmp.general.isoncluster = false;
  % Set_tmp.general.parcomp_maxworkers = 0; % Bei lokaler Ausführung ohne ParPool
  % Set_tmp.general.compile_missing_functions = false; % lokales Testen, damit es schneller geht
  % Set_tmp.general.update_template_functions = false; % lokales Testen
  % Set_tmp.general.check_missing_template_functions = false; % damit schneller
  % Überschreibe das Verzeichnis, das in den Einstellungen gesetzt ist.
  % Dadurch auch lokaler Abschluss, wenn vom Cluster heruntergeladen.
  % TODO: Eigentlich gibt es dafür schon eine Logik in cds_start, die
  % aber scheinbar nicht funktioniert.
  Set_tmp.optimization.resdir = respath;
  if any(~complete & tmpfiles_available)
    fprintf('Optimierung %s ist unfertig. Schließe vorläufiges Ergebnis ab\n', ...
      optdirs(i).name);
    cds_start(Set_tmp, sd.Traj); % erzeugt auch die Tabelle neu
  elseif any(complete) && ~exist(tf_file, 'file')
    % nur Tabelle neu erzeugen
    Set_tmp.general.only_finish_aborted = false;
    Set_tmp.general.regenerate_summary_only = true;
    cds_start(Set_tmp, sd.Traj);
  end
end
