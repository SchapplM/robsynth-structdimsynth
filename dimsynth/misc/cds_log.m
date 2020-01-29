% Log-Funktion für die Maßsynthese
% Speichert den in Konsole ausgegebenen Text in Datei
% 
% Eingabe:
% level
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

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-01
% (C) Institut für Mechatronische Systeme, Universität Hannover

function cds_log(level, msg, option, Set, Structure)
% Initialisierung
persistent logfilepath
persistent loglevel
if nargin > 2 && strcmp(option, 'init')
  % Initialisiere die persistenten Variablen
  resdir = fullfile(Set.optimization.resdir, Set.optimization.optname);
  logfilepath = fullfile(resdir, sprintf('Rob%d_%s.log', Structure.Number, Structure.Name));
  loglevel = Set.general.verbosity;
  if exist(logfilepath, 'file')
    movefile(logfilepath, fullfile(resdir, sprintf('Rob%d_%s_backup_%s.log', ...
      Structure.Number, Structure.Name, datestr(now,'yyyymmdd_HHMMSS'))) );
  end
end
% Log-Zeile abspeichern
timestr = datestr(now,'yyyy-mm-dd HH:MM:SS');
fid = fopen(logfilepath, 'a');
fprintf(fid, '[%s] %s\n', timestr, msg);
% Nur in Konsole ausgeben, wenn Nachricht wichtig genug ist
if level <= loglevel
  fprintf('%s\n', msg);
end
fclose(fid);