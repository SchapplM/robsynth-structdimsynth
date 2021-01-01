% Fasse für das Cluster aufgeteilte Ergebnisse zusammen
% Notwendig, wenn Set.general.cluster_maxrobotspernode benutzt wurde
% 
% Eingabe:
% optname
%   Name der Optimierung. Entspricht Eintrag in Set.optimization.optname.
%   (die Ordner der Teilergebenisse mit Endung _p1, _p2, ... werden darin
%   zusammengefasst).
% mode
%   Modus, mit dem die Daten zusammengefasst werden:
%   'copy': Kopiere die Daten aus den Part-Ordnern in den neuen Ordnern
%   'move': Verschiebe die Daten
%   'symlink': Erzeuge symbolische Verknüpfungen im Dateisystem (sinnvoll,
%   falls die Ursprungsdaten mit dem Cluster synchronisiert werden und
%   Änderungen/Umbenennungen durch erneute Synchronisation überschrieben
%   würde)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-11
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function cds_merge_results(optname, mode, create_missing_tables, create_pareto_fig)

if nargin < 2
  mode = 'copy';
end
if nargin < 3
  create_missing_tables = false;
end

resdir = fullfile(fileparts(which('structgeomsynth_path_init.m')), 'results');
resdir_ges = fullfile(resdir, optname);

if exist(resdir_ges, 'file')
%   fprintf('Gesamt-Verzeichnis %s existiert schon. Abbruch.\n', resdir_ges);
%   return
  rmdir(resdir_ges, 's');
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
  tokens = regexp(optdirs(i).name, '.*?_p([\d]+)', 'tokens');
  maxnum_parts = max(maxnum_parts, str2double(tokens{1}{1}));
end
numdirs_processed = 0;
% Gehe durch alle (möglichen) Ergebnis-Ordner
for i = 1:maxnum_parts
  % Erzeuge den Ordner-Namen hier neu, da nicht davon ausgegangen werden
  % kann, dass die Reihenfolge mit aufsteigenden Nummern ist.
  dirname_i = sprintf('%s_p%d', optname, i);
  II = find(strcmp({optdirs(:).name}, dirname_i),1);
  if isempty(II)
    warning('Ordner %s nicht gefunden', dirname_i);
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
    if create_missing_tables
      cds_results_table(s.Set, s.Traj, s.Structures);
    else
      warning('Ergebnisse im Ordner %s sind unvollständig. Keine csv-Tabelle. Überspringe.', dirname_i);
      continue % Ignoriere diesen Ordner
    end
  end
  % Erzeuge die Gesamt-Variable der Roboterstrukturen für die neue
  % Einstellungsdatei.Prüfe, welche Nummerierung in der Einstellungsdatei ist.
  IdxAlt = 1:length(s.Structures);
  if i == 1
    Structures = s.Structures;
    IdxNeu = IdxAlt;  % Nr. nicht verändern
  else
    IdxNeu = IdxAlt + Structures{end}.Number; % alle Nummern hochzählen.
    % Nummer der Roboter in den Strukturen verändern
    for kk = 1:length(s.Structures)
      s.Structures{kk}.Number = IdxNeu(s.Structures{kk}.Number);
    end
    Structures = [Structures, s.Structures]; %#ok<AGROW>
  end
  % Tabelle zusammenstellen (aus den Tabellen der Teil-Ergebnisse)
  ResTab_i = readtable(csvtable_i, 'HeaderLines', 2);
  if isempty(ResTab_i)
    warning('Ergebnis-Tabelle %s existiert, ist aber leer.', csvtable_i);
    if create_missing_tables
      cds_results_table(s.Set, s.Traj, s.Structures);
    end
    continue % Ignoriere diesen Ordner
  end
  ResTab_i_headers = readtable(csvtable_i, 'ReadVariableNames', true);
  ResTab_i.Properties.VariableNames = ResTab_i_headers.Properties.VariableNames;
  if i == 1 % Setze als Gesamt-Tabelle
    ResTab_ges = ResTab_i;
  else % Hänge Tabelle an
    % Erhöhe die Laufende Nummer der Roboter in der Tabelle. Annahme: Die
    % Roboter in jeder Tabelle fangen bei Nr. 1 an. Ziel ist, dass die
    % Roboter in der Gesamt-Tabelle fortlaufend nummeriert sind.
    ResTab_i.LfdNr(:) = IdxNeu(ResTab_i.LfdNr);
    % Hänge die aktuelle Tabelle (mit nur einem Teil der Ergebnisse) an das
    % Ende der Gesamt-Tabelle an)
    ResTab_ges = [ResTab_ges; ResTab_i]; %#ok<AGROW>
  end
  % Kopiere die Dateien für den Roboter in den Ordner der Gesamt-Ergebnisse.
  % Ändere dabei die Nummer der Roboter. Das Namensschema der Ergebnisse
  % wird in cds_start.m und cds_dimsynth_robot.m bestimmt.
  resobjects_i = dir(fullfile(resdir,dirname_i,'Rob*'));
  for j = 1:length(resobjects_i)
    if resobjects_i(j).isdir % Ist ein Ordner mit Bildern/Animation/Log-Datei
      tokens = regexp(resobjects_i(j).name, 'Rob([\d]+)_([A-Z0-9]+)', 'tokens');
    else % Ist eine Endergebnis-mat-Datei
      tokens = regexp(resobjects_i(j).name, 'Rob([\d]+)_([A-Z0-9]+)_Endergebnis.mat', 'tokens');
      if isempty(tokens)
        tokens = regexp(resobjects_i(j).name, 'Rob([\d]+)_([A-Z0-9]+)_Details.mat', 'tokens');
      end
      if isempty(tokens)
        error('Datei %s passt nicht in das Suchschema', resobjects_i(j).name);
      end
    end
    % Lese Daten des Roboters aus den Datei-/Ordnernamen
    RobNr_j_alt = str2double(tokens{1}{1});
    RobName_j = tokens{1}{2};
    RobNr_j_neu = IdxNeu(IdxAlt==RobNr_j_alt); % Setze neue Nummer
    if resobjects_i(j).isdir % Kopiere das Verzeichnis
      dirname_j_neu = sprintf('Rob%d_%s', RobNr_j_neu, RobName_j);
      if strcmp(mode, 'copy')
        copyfile(fullfile(resdir,dirname_i,resobjects_i(j).name), ...
          fullfile(resdir_ges, dirname_j_neu));
      elseif strcmp(mode, 'move')
        movefile(fullfile(resdir,dirname_i,resobjects_i(j).name), ...
          fullfile(resdir_ges, dirname_j_neu));
      elseif strcmp(mode, 'symlink')
        if ~isunix()
          error('Symbolische Verknüpfung nur unter Linux möglich');
        end
        % Erstelle den Ordner, damit nicht der Ordner selbst verlinkt wird,
        % sondern alle darin enthaltenen Dateien
        mkdirs(fullfile(resdir_ges, dirname_j_neu));
        % Verlinke alle Dateien im Verzeichnis
        resfiles_j = dir(fullfile(resdir,dirname_i,resobjects_i(j).name, 'Rob*.*'));
        for k = 1:length(resfiles_j)
          tokens_k = regexp(resfiles_j(k).name, sprintf('Rob%d_%s(.*)',RobNr_j_alt,RobName_j), 'tokens');
          if isempty(tokens_k)
            warning('Datei %s entspricht nicht dem Namensschema', resfiles_j(k).name);
            continue
          end
          newfilename_k = sprintf('Rob%d_%s%s', RobNr_j_neu, RobName_j, tokens_k{1}{1});
          system(sprintf('ln -s %s %s', fullfile(resdir,dirname_i,resobjects_i(j).name,resfiles_j(k).name), ...
            fullfile(resdir_ges,dirname_j_neu,newfilename_k)));
        end
      else
        error('Modus %s nicht definiert', mode);
      end
      % Benenne alle Dateien in den Ordnern konsistent um
      if RobNr_j_alt ~= RobNr_j_neu && ~strcmp(mode, 'symlink')
        % Keine Umbenennung notwendig, wenn symbolische Verknüpfungen
        % gesetzt wurden. Dann ist der neue Name schon korrekt.
        resfiles_j = dir(fullfile(resdir_ges, dirname_j_neu, 'Rob*.*'));
        for k = 1:length(resfiles_j)
          tokens_k = regexp(resfiles_j(k).name, sprintf('Rob%d_%s(.*)',RobNr_j_alt,RobName_j), 'tokens');
          if isempty(tokens_k)
            warning('Datei %s entspricht nicht der Erwartung.', resfiles_j(k).name);
            continue
          end
          newfilename_k = sprintf('Rob%d_%s%s', RobNr_j_neu, RobName_j, tokens_k{1}{1});
          movefile(fullfile(resdir_ges,dirname_j_neu,resfiles_j(k).name), ...
                   fullfile(resdir_ges,dirname_j_neu,newfilename_k));
        end
      end
    else % Kopiere die Endergebnis-mat-Dateien
      if contains(resobjects_i(j).name, 'Endergebnis.mat')
        resfilename_j_neu = sprintf('Rob%d_%s_Endergebnis.mat', RobNr_j_neu, RobName_j);
      elseif contains(resobjects_i(j).name, 'Details.mat')
        resfilename_j_neu = sprintf('Rob%d_%s_Details.mat', RobNr_j_neu, RobName_j);
      else
        error('Unbehandelte Ausnahme');
      end
      sourcefile = fullfile(resdir,dirname_i,resobjects_i(j).name);
      targetfile = fullfile(resdir_ges, resfilename_j_neu);
      if strcmp(mode, 'copy')
        copyfile(sourcefile, targetfile);
      elseif strcmp(mode, 'move') % Verschieben anstatt kopieren
        movefile(sourcefile, targetfile);
      elseif strcmp(mode, 'symlink')
        if ~isunix()
          error('Symbolische Verknüpfung nur unter Linux möglich');
        end
        system(sprintf('ln -s "%s" "%s"', sourcefile, targetfile));
      else
        error('Modus %s nicht definiert', mode);
      end
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
writetable(ResTab_ges, csvtable_ges, 'Delimiter', ';');
% Speichere die Gesamt-Einstellungsdatei ab (wird für einige Auswertungs-
% Skripte benötigt).
settingsfile = fullfile(resdir_ges,[optname, '_settings.mat']);
Set = s.Set;
Set.optimization.optname = optname;
Traj = s.Traj;
save(settingsfile, 'Structures', 'Traj', 'Set');
% Pareto-Bild erstellen (ist nur für die einzelnen Teile generiert worden)
if create_pareto_fig && length(Set.optimization.objective) > 1
  Set.general.eval_figures = {'pareto_all'};
  Set.general.animation_styles = {};
  cds_vis_results(Set, Traj, Structures);
end