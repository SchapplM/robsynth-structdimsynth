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
  end
  if ~any(~complete & tmpfiles_available)
    % Es gibt keine Wiederherstellungsdateien, die helfen würden
    fprintf('%s: Keine Wiederherstellungsdateien\n', optdirs(i).name);
    continue
  end
  %% Schließe die Optimierung erneut ab
  fprintf('Optimierung %s ist unfertig. Schließe vorläufiges Ergebnis ab\n', ...
    optdirs(i).name);
  Set_tmp = cds_settings_update(sd.Set, 1);
  Set_tmp.computing_cluster = false; % Abschluss muss immer lokal gemacht werden bezogen auf System, das dieses Skript hier ausführt
  Set_tmp.general.only_finish_aborted = true;
  cds_start(Set_tmp, sd.Traj);
end
