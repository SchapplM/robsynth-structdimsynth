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
%   Für Zurücksetzen der Log-Datei: "init". Sonst keine Eingabe
% Set, Structure
%   Informationsstrukturen für Optimierung (siehe andere Dateien)
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
% Initialisierung durch Funktionsaufruf
if nargin > 2 && strcmp(option, 'init')
  % Initialisiere die persistenten Variablen
  resdir = fullfile(Set.optimization.resdir, Set.optimization.optname);
  robstr = sprintf('Rob%d_%s', Structure.Number, Structure.Name);
  logfilepath = fullfile(resdir, robstr, sprintf('%s.log', robstr));
  loglevel = Set.general.verbosity;
  if exist(logfilepath, 'file')
    movefile(logfilepath, fullfile(resdir, robstr, sprintf('%s_backup_%s.log', ...
      robstr, datestr(now,'yyyymmdd_HHMMSS'))) );
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
  if level > 0
    fprintf('%s\n', msg);
  else
    warning(msg);
  end
end
lfp = logfilepath;
