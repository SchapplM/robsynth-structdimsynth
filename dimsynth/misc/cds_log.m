% Log-Funktion für die Maßsynthese eines einzelnen Roboters
% Speichert den in Konsole ausgegebenen Text in Datei
% 
% Eingabe:
% level
%  -1=Warnung (immer anzeigen)
%   0=minimal (Endergebnis)
%   1=mehr (Generationenweise)
%   2=noch mehr (Jede Fitness-Eval.)
%   3=alles in Fitness
%   4=Auch Entwurfsoptimierung
% msg
%   Log-Zeile
% option
%   Für Zurücksetzen der Log-Datei: "init"
%   Für erneute Initialisierung ohne Löschen der Datei: "amend"
%   Sonst keine Eingabe
% Set, Structure
%   Informationsstrukturen für Optimierung (siehe andere Dateien)
%   Falls Structure.Number = 0: Allgemeine Log-Datei
% 
% Ausgabe:
% lfp
%   Pfad zur Log-Datei

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function lfp = cds_log(level, msg, option, Set, Structure)
%% Initialisierung
persistent logfilepath
persistent loglevel
if nargin == 0
  logfilepath = [];
  loglevel = [];
  return
end
% Initialisierung durch Funktionsaufruf
if nargin > 2 && any(strcmp(option, {'init', 'amend'}))
  % Initialisiere die persistenten Variablen
  resdir = fullfile(Set.optimization.resdir, Set.optimization.optname);
  loglevel = Set.general.verbosity;
  if nargin < 5 || isempty(Structure)  % Falls ohne Struktur-Info ...
    return % ... initialisiert wird, wird nur das Log-Level neu gesetzt
  end
  if Structure.Number > 0
    % Roboterspezifische Log-Datei
    robstr = sprintf('Rob%d_%s', Structure.Number, Structure.Name);
    logfilepath = fullfile(resdir, robstr, sprintf('%s.log', robstr));
  else
    % Allgemeine Log-Datei (unterschieden in Hochladen und Optimierung auf
    % Cluster). Bei Hochladen keine Datei erstellen.
    if Set.general.computing_cluster
      robstr = '';
      logfilepath = '';
    else
      % Je nach Modus unterschiedliche Log-Dateien
      if Set.general.only_finish_aborted
        robstr = 'cds_finish';
      elseif Set.general.regenerate_summary_only
        robstr = 'cds_summary';
      else
        robstr = 'cds_dimsynth';
      end
      logfilepath = fullfile(resdir, [robstr,'.log']);
    end
  end
  if ~isempty(logfilepath) && exist(logfilepath, 'file') && strcmp(option, 'init')
    backupfilename = sprintf('%s_backup_%s.log', robstr, ...
      datestr(now,'yyyymmdd_HHMMSS'));
    logdir = fileparts(logfilepath);
    movefile(logfilepath, fullfile(logdir, backupfilename));
    % Log-Datei komprimieren (verbleibt sonst unkomprimiert im Ordner)
    gzip(fullfile(logdir, backupfilename)); delete(fullfile(logdir, backupfilename));
  end
end
% Falls Fitness-Funktion nachträglich aufgerufen wird, ist das Loggen nicht
% mehr initialisiert. Temporäre Belegung der Variablen in diesem Fall
if isempty(loglevel)
  loglevel = 5;
end
%% Schreiben in Datei
if ~isempty(logfilepath) % Bei Aufruf der Funktion aus mat-Datei ohne Init.
  % Log-Zeile abspeichern
  timestr = datestr(now,'yyyy-mm-dd HH:MM:SS');
  fid = fopen(logfilepath, 'a');
  fprintf(fid, '[%s] %s\n', timestr, msg);
  fclose(fid);
end
%% Schreiben in Konsole
% Nur in Konsole ausgeben, wenn Nachricht wichtig genug ist
if level <= loglevel
  if level > -1
    fprintf('%s\n', msg);
  else
    warning(msg);
  end
end
lfp = logfilepath;
