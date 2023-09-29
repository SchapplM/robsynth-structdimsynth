% Fasse für das Cluster aufgeteilte Ergebnisse zusammen
% Notwendig, wenn Set.general.cluster_maxrobotspernode benutzt wurde
% 
% Eingabe:
% optname
%   Name der Optimierung. Entspricht Eintrag in Set.optimization.optname.
%   (die Ordner der Teilergebenisse mit Endung _p1, _p2, ... werden darin
%   zusammengefasst).
% settings
%   Struktur mit Einstellungen. Felder:
% .mode
%   Modus, mit dem die Daten zusammengefasst werden:
%   'copy': Kopiere die Daten aus den Part-Ordnern in den neuen Ordnern
%   'move': Verschiebe die Daten
%   'symlink': Erzeuge symbolische Verknüpfungen im Dateisystem (sinnvoll,
%   falls die Ursprungsdaten mit dem Cluster synchronisiert werden und
%   Änderungen/Umbenennungen durch erneute Synchronisation überschrieben
%   würde)
% .create_missing_tables
%   Fehlende Ergebnis-Tabellen erstellen
% .create_pareto_fig
%   Neues Pareto-Bild mit den zusammengefassten Ergebnissen erstellen
% .delete_parts_dirs
%   Lösche die Ordner mit den Teil-Ergebnissen nach erfolgreichem Merge
% .retry
%   Versuche das Zusammenführen erneut, z.B. falls vorher unvollständig

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-11
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function cds_merge_results(optname, settings_in)

% Bedeutung der Einstellungen, siehe Kommentar oben
settings = struct( ...
  'mode', 'copy', ...
  'create_missing_tables', false, ...
  'create_pareto_fig', false, ...
  'delete_parts_dirs', true, ...
  'resdir', '', ... % Standardmäßig Ordner im Repo nehmen. Optional anderes
  'retry', false);
if nargin == 2
  for f = fields(settings_in)'
    if ~isfield(settings, f{1})
      warning('Unerwartetes Feld %s in Eingabe. Ignorieren.');
      continue
    end
    settings.(f{1}) = settings_in.(f{1});
  end
end
if strcmp(settings.mode, 'symlink')
  settings.delete_parts_dirs = false; % Bei Verlinkung löschen nicht sinnvoll.
end
if isempty(settings.resdir)
  resdir = fullfile(fileparts(which('structgeomsynth_path_init.m')), 'results');
else
  resdir = settings.resdir;
end
resdir_ges = fullfile(resdir, optname);

if exist(resdir_ges, 'file') 
  if ~settings.retry
    fprintf('Gesamt-Verzeichnis %s existiert schon. Abbruch.\n', resdir_ges);
    return
  else
    % Benenne das Ziel-Verzeichnung um als Part 0. Damit wird diese wieder
    % mit den anderen Teilen zusammengeführt.
    resdir_p0 = fullfile(resdir, [optname, '_p0']);
    if exist(resdir_p0, 'file')
      fprintf('Verzeichnis %s existiert schon. Abbruch\n', resdir_p0);
      return;
    end
    movefile(resdir_ges, resdir_p0);
    % Dateien umbenennen, damit konsistent mit erwartetem Schema
    if ~exist(fullfile(resdir_p0, [optname, '_settings.mat']), 'file')
      fprintf('Datei %s existiert nicht. Unerwarteter Fehler\n', ...
        fullfile(resdir_p0, [optname, '_settings.mat']));
      return
    end
    movefile(fullfile(resdir_p0, [optname, '_settings.mat']), ...
             fullfile(resdir_p0, [optname, '_p0_settings.mat']));
    if ~exist(fullfile(resdir_p0, [optname, '_results_table.csv']), 'file')
      fprintf('Datei %s existiert nicht. Unerwarteter Fehler\n', ...
        fullfile(resdir_p0, [optname, '_results_table.csv']));
      return
    end
    movefile(fullfile(resdir_p0, [optname, '_results_table.csv']), ...
             fullfile(resdir_p0, [optname, '_p0_results_table.csv']));
  end
  % rmdir(resdir_ges, 's');
end
% Zielordner erstellen
mkdirs(resdir_ges);

% Finde alle aufgeteilten Ergebnisse
optdirs = dir(fullfile(resdir, [optname, '_p*']));
if isempty(optdirs)
  warning('Keine aufgeteilten Ergebnis-Ordner "%s" in %s vorhanden', optname, resdir);
  return
end
% Prüfe die maximale Nummer der Ordner (es kann auch eine Nummer vom Anfang
% fehlen)
maxnum_parts = 0;
for i = 1:length(optdirs)
  tokens = regexp(optdirs(i).name, '.*?_p([\d]+)$', 'tokens');
  if isempty(tokens)
    % Der Ordner passt nicht ins Namensschema. Eventuell ist das Ende nicht
    % p1,p2,p5,..., sondern etwas anderes, wie z.B. "_perfmap"
    continue
  end
  maxnum_parts = max(maxnum_parts, str2double(tokens{1}{1}));
end
numdirs_processed = 0;
dir_success = false(1,1+maxnum_parts);
% Gehe durch alle (möglichen) Ergebnis-Ordner
for i = 0:maxnum_parts % Ordner 0 kommt nicht aus Optimierung sondern von oben
  % Erzeuge den Ordner-Namen hier neu, da nicht davon ausgegangen werden
  % kann, dass die Reihenfolge mit aufsteigenden Nummern ist.
  dirname_i = sprintf('%s_p%d', optname, i);
  II = find(strcmp({optdirs(:).name}, dirname_i),1);
  if isempty(II)
    if i > 0 % Ordner 0 existiert nur in seltenen Fällen
      warning('Ordner %s nicht gefunden', dirname_i);
    end
    continue; % fehlender Ordner. Versuche den nächsten
  end
  % Lade die Einstellungen
  settingsfile_i = fullfile(resdir,dirname_i,[dirname_i, '_settings.mat']);
  if ~exist(settingsfile_i, 'file')
    warning(['Einstellungsdatei %s fehlt. Ordner %s ist stark unvollständig. ', ...
      'Möglicher Fehler.'], settingsfile_i, dirname_i);
    continue;
  end
  s=load(settingsfile_i, 'Set', 'Traj', 'Structures');
  s.Set.optimization.resdir = resdir;
  % Lese die aktuelle Tabelle
  csvtable_i = fullfile(resdir,dirname_i,[dirname_i,'_results_table.csv']);
  if ~exist(csvtable_i, 'file')
    if settings.create_missing_tables
      cds_results_table(s.Set, s.Traj, s.Structures);
    else
      warning('Ergebnisse im Ordner %s sind unvollständig. Keine csv-Tabelle. Überspringe.', dirname_i);
      continue % Ignoriere diesen Ordner
    end
  end
  % Erzeuge die Gesamt-Variable der Roboterstrukturen für die neue
  % Einstellungsdatei. Nummerierung ist schon richtig.
  if numdirs_processed == 0 % statt `i==1`, falls i=1 nicht existiert
    Structures = s.Structures;
  else
    Structures = [Structures, s.Structures]; %#ok<AGROW>
  end
  % Tabelle zusammenstellen (aus den Tabellen der Teil-Ergebnisse)
  opts = detectImportOptions(csvtable_i,'NumHeaderLines',2);
  opts.VariableNamesLine = 1;
  opts.VariableDescriptionsLine = 2;
  ResTab_i = readtable(csvtable_i, opts);
  if isempty(ResTab_i)
    warning('Ergebnis-Tabelle %s existiert, ist aber leer.', csvtable_i);
    if settings.create_missing_tables
      cds_results_table(s.Set, s.Traj, s.Structures);
    end
    continue % Ignoriere diesen Ordner
  end
  % Prüfe Erfolg des Teils anhand der Ergebnis-Tabelle
  if size(ResTab_i,1) == length(s.Structures)
    dir_success(1+i) = true;
  else
    warning(['Optimierung Teil %d war nicht erfolgreich: %d Zeilen in Ergebnis, ', ...
      '%d erwartet.'], i, size(ResTab_i,1), length(s.Structures));
  end
  if numdirs_processed == 0 % Setze als Gesamt-Tabelle. Das hier ist das erste vollständige Ergebnis mit Tabelle
    ResTab_ges = ResTab_i;
  else % Hänge Tabelle an
    % Hänge die aktuelle Tabelle (mit nur einem Teil der Ergebnisse) an das
    % Ende der Gesamt-Tabelle an). Nummern passen bereits.
    ResTab_ges = [ResTab_ges; ResTab_i]; %#ok<AGROW>
  end
  % Kopiere die Dateien für den Roboter in den Ordner der Gesamt-Ergebnisse.
  % Ändere dabei die Nummer der Roboter. Das Namensschema der Ergebnisse
  % wird in cds_start.m und cds_dimsynth_robot.m bestimmt.
  resobjects_i = dir(fullfile(resdir,dirname_i,'Rob*'));
  for j = 1:length(resobjects_i)
    % Egal ob Ergebnisordner eines Roboters oder Ergebnis-Datei. Muster:
    tokens = regexp(resobjects_i(j).name, 'Rob([\d]+)_([A-Z0-9]+)(.*)', 'tokens');
    if isempty(tokens)
      warning('Datei %s passt nicht in das Suchschema', resobjects_i(j).name);
      continue
    end
    % Lese Daten des Roboters aus den Datei-/Ordnernamen
    RobNr_j = str2double(tokens{1}{1});
    RobName_j = tokens{1}{2};
    if length(tokens{1}) == 3
      Suffix = tokens{1}{3};
    else
      Suffix = '';
    end
    if resobjects_i(j).isdir % Kopiere das Verzeichnis
      dirname_j_neu = sprintf('Rob%d_%s', RobNr_j, RobName_j);
      mode = settings.mode;
      done = false;
      if strcmp(mode, 'move')
        try % Bei Windows Verschieben evtl. nicht möglich wegen offener Dateien
          movefile(fullfile(resdir,dirname_i,resobjects_i(j).name), ...
            fullfile(resdir_ges, dirname_j_neu));
          done = true;
        catch err
          warning(sprintf(['Verschieben nicht möglich. Kopiere ' ...
            'stattdessen: %s'], err.message)); %#ok<SPWRN> 
          mode = 'copy';
        end
      end
      if strcmp(mode, 'copy')
        copyfile(fullfile(resdir,dirname_i,resobjects_i(j).name), ...
          fullfile(resdir_ges, dirname_j_neu));
        done = true;
      end
      if strcmp(mode, 'symlink')
        if ~isunix()
          error('Symbolische Verknüpfung nur unter Linux möglich');
        end
        % Erstelle den Ordner, damit nicht der Ordner selbst verlinkt wird,
        % sondern alle darin enthaltenen Dateien
        mkdirs(fullfile(resdir_ges, dirname_j_neu));
        % Verlinke alle Dateien im Verzeichnis
        resfiles_j = dir(fullfile(resdir,dirname_i,resobjects_i(j).name, 'Rob*.*'));
        for k = 1:length(resfiles_j)
          tokens_k = regexp(resfiles_j(k).name, sprintf('Rob%d_%s(.*)',RobNr_j,RobName_j), 'tokens');
          if isempty(tokens_k)
            warning('Datei %s entspricht nicht dem Namensschema', resfiles_j(k).name);
            continue
          end
          newfilename_k = sprintf('Rob%d_%s%s', RobNr_j, RobName_j, tokens_k{1}{1});
          system(sprintf('ln -s %s %s', fullfile(resdir,dirname_i,resobjects_i(j).name,resfiles_j(k).name), ...
            fullfile(resdir_ges,dirname_j_neu,newfilename_k)));
        end
        done = true;
      end
      if ~done
        error('Modus %s nicht definiert oder nicht erfolgreich', settings.mode);
      end
    else % Kopiere die Endergebnis-mat-Dateien
      resfilename_j_neu = sprintf('Rob%d_%s%s', RobNr_j, RobName_j, Suffix);
      sourcefile = fullfile(resdir,dirname_i,resobjects_i(j).name);
      targetfile = fullfile(resdir_ges, resfilename_j_neu);
      mode = settings.mode;
      done = false;
      if strcmp(settings.mode, 'move') % Verschieben anstatt kopieren
        try
          movefile(sourcefile, targetfile);
          done = true;
        catch err
          warning(sprintf(['Verschieben nicht möglich. Kopiere ' ...
            'stattdessen: %s'], err.message)); %#ok<SPWRN> 
          mode = 'copy';
        end
      end
      if strcmp(mode, 'copy')
        copyfile(sourcefile, targetfile);
        done = true;
      end
      if strcmp(mode, 'symlink')
        if ~isunix()
          error('Symbolische Verknüpfung nur unter Linux möglich');
        end
        system(sprintf('ln -s "%s" "%s"', sourcefile, targetfile));
        done = true;
      end
      if ~done
        error('Modus %s nicht definiert', settings.mode);
      end
    end
  end
  % Kopiere auch den Inhalt des tmp-Ordners
  tmpdirs = dir(fullfile(resdir, dirname_i, 'tmp', '*_*'));
  for k = 1:length(tmpdirs)
    tokens = regexp(tmpdirs(k).name, '([\d]+)_([A-Z0-9]+)', 'tokens');
    if isempty(tokens) || ~tmpdirs(k).isdir
      continue
    end
    % Lese Daten des Roboters aus den Datei-/Ordnernamen
    RobNr_j = str2double(tokens{1}{1});
    RobName_j = tokens{1}{2};
    tmpdir_j_alt = fullfile(resdir, dirname_i, 'tmp', sprintf('%d_%s', ...
      RobNr_j, RobName_j));
    tmpdir_j_neu = fullfile(resdir_ges, 'tmp', sprintf('%d_%s', ...
      RobNr_j, RobName_j));
    % Prüfe, ob das Verzeichnis nicht leer ist
    tmpobjects_j = dir(tmpdir_j_alt);
    emptydir = true;
    for l = 1:length(tmpobjects_j)
      if tmpobjects_j(l).name(1) == '.', continue; end
      emptydir = false; break;
    end
    if emptydir, continue; end
    % Gesamt-Verzeichnis erstellen
    mkdirs(fullfile(resdir_ges, 'tmp'));
    mode = settings.mode;
    done = false;
    % Kopiere das tmp-Verzeichnis
    if strcmp(mode, 'move')
      try
        movefile(tmpdir_j_alt, tmpdir_j_neu);
        done = true;
      catch err
        warning(sprintf(['Verschieben von %s nicht möglich: %s. Kopiere ' ...
          'stattdessen'], tmpdir_j_alt, err.message)); %#ok<SPWRN>
        mode = 'copy';
      end
    end
    if strcmp(mode, 'copy')
      copyfile(tmpdir_j_alt, tmpdir_j_neu);
      done = true;
    end
    if strcmp(mode, 'symlink')
      if ~isunix()
        error('Symbolische Verknüpfung nur unter Linux möglich');
      end
      % Erstelle den Ordner, damit nicht der Ordner selbst verlinkt wird,
      % sondern alle darin enthaltenen Dateien
      mkdirs(tmpdir_j_neu);
      % Verlinke alle Dateien im Verzeichnis
      for l = 1:length(tmpobjects_j)
        if tmpobjects_j(l).name(1) == '.', continue; end
        system(sprintf('ln -s %s %s', fullfile(tmpdir_j_alt,tmpobjects_j(l).name), ...
          fullfile(tmpdir_j_neu,tmpobjects_j(l).name)));
      end
      done = true;
    end
    if ~done
      error('Modus %s nicht definiert oder nicht erfolgreich.', settings.mode);
    end
  end
  % Kopiere die Log-Dateien
  logfiles_i = dir(fullfile(resdir, dirname_i,'*.log*'));
  mkdirs(fullfile(resdir_ges, 'tmp', 'logs'));
  for j = 1:length(logfiles_i)
    [tokens, ~] = regexp(logfiles_i(j).name, '(.*).(log.*)', 'tokens', 'match');
    if isempty(tokens)
      warning('Muster konnte in Datei %s nicht gefunden werden', logfiles_i(j).name);
      continue
    end
    filename_new = sprintf('%s_p%d.%s', tokens{1}{1}, i, tokens{1}{2});
    sourcefile = fullfile(resdir, dirname_i, logfiles_i(j).name);
    targetfile = fullfile(resdir_ges, 'tmp', 'logs', filename_new);
    if strcmp(settings.mode, 'copy')
      copyfile(sourcefile, targetfile);
    elseif strcmp(settings.mode, 'move')
      movefile(sourcefile, targetfile);
    elseif strcmp(settings.mode, 'symlink')
      if ~isunix()
        error('Symbolische Verknüpfung nur unter Linux möglich');
      end
      system(sprintf('ln -s "%s" "%s"', sourcefile, targetfile));
    else
      error('Modus %s nicht definiert', settings.mode);
    end
  end
  fprintf('%s: Teil-Ergebnis %d/%d kopiert.\n', optname, i, length(optdirs));
  numdirs_processed = numdirs_processed + 1;
end

if numdirs_processed == 0
  warning('Keine Ergebnisse zu %s gefunden', optname);
  return
end
% Speichere die Gesamt-Tabelle ab
csvtable_ges = fullfile(resdir_ges,[optname,'_results_table.csv']);
% Zeile mit Überschriften und erklärenden Kommentaren zuerst
Tab_Descr  = cell2table(ResTab_ges.Properties.VariableDescriptions, ...
  'VariableNames', ResTab_ges.Properties.VariableNames);
writetable(Tab_Descr, csvtable_ges, 'Delimiter', ';');
% Eigentlichen Tabelleninhalt schreiben (Überschriften sind nicht doppelt)
writetable(ResTab_ges, csvtable_ges, 'Delimiter', ';', 'WriteMode', 'append');
% Speichere die Gesamt-Einstellungsdatei ab (wird für einige Auswertungs-
% Skripte benötigt).
Set = s.Set;
% Setze den ursprünglichen Namen der Optimierung ohne Bezeichnung p_...
Set.optimization.optname = optname;
% Entferne die Filterung der Roboter, die in den Einstellungen der
% Einstellungen der Teile enthalten ist (erzeugt später komische Ausgabe).
% Erzeuge neuen Filter für alle enthaltenen Roboter
Set.structures.whitelist = ResTab_ges.Name';
Traj = s.Traj;
settingsfile = fullfile(resdir_ges,[optname, '_settings.mat']);
save(settingsfile, 'Structures', 'Traj', 'Set');
% Pareto-Bild erstellen (ist nur für die einzelnen Teile generiert worden)
if settings.create_pareto_fig && length(Set.optimization.objective) > 1
  Set.general.eval_figures = {'pareto_all_phys'};
  Set.general.animation_styles = {};
  cds_vis_results(Set, Traj, Structures);
end

% Lösche die Ordner mit den Teil-Ergebnissen (werden nicht mehr benötigt)
if settings.delete_parts_dirs
  for i = find(dir_success)-1
    dirname_i = sprintf('%s_p%d', optname, i);
    fprintf('Verzeichnis %d erfolgreich zusamengeführt. Lösche %s\n', i, dirname_i);
    rmdir(fullfile(resdir, dirname_i), 's');
  end
end
