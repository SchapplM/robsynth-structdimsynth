% Initialisiere den Matlab-ParPool mit Einstellungen aus der Maßsynthese
% Benutze viele try-catch-Befehle aufgrund von Fehlern auf dem Cluster.
% 
% Eingabe:
% Set
%   Struktur aus Maßsynthese. Siehe cds_start und cds_settings_defaults
% 
% Ausgabe:
% parfor_numworkers
%   Anzahl der tatsächlich gestarteten ParPool-Worker.
%   * 0: In der parfor-Schleife (als zweites Argument verwendet) wird kein
%        ParPool benutzt, sondern eine normale for-Schleife
%   * 1: Wird nicht verwendet. Das wäre eine parfor-Schleife auf einem Kern.
%        Bringt keinen Vorteil (außer dass Bilder unsichtbar erzeugt werde
%   * >1: Normale parfor-Schleife mit so vielen Parallelinstanzen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-04
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function parfor_numworkers = cds_start_parpool(Set)
if Set.general.parcomp_maxworkers <= 1
  parfor_numworkers = 0;
  return;
end
t0 = tic();
if Set.general.isoncluster % auf Cluster möglicher Zugriffskonflikt für ParPool
  parpool_writelock('lock', 180, true); % Synchronisationsmittel für ParPool
end
for i = 1:5 % Versuche mehrfach, den Pool zu starten
  if i > 1 % Zufällige Wartezeit zur Prävention von Thread-Konflikten
    cds_log(1, sprintf('[start_parpool] Erneuter Versuch %d nach Fehler', i));
    pause(5+(5+i)*rand());
  end
  Pool = gcp('nocreate');
  if isempty(Pool)
    try
      cds_log(1, sprintf(['[start_parpool] Starte ParPool mit Ziel ', ...
        'parfor_numworkers=%d. Zeit seit Funktionsaufruf: %1.1fs'], ...
        Set.general.parcomp_maxworkers, toc(t0)));
      Pool=parpool([1,Set.general.parcomp_maxworkers]);
      parfor_numworkers = Pool.NumWorkers;
    catch err
      cds_log(-1, sprintf(['[start_parpool] Fehler beim Starten des parpool ', ...
        '(%1.1fs nach Funktionsaufruf): %s.'], toc(t0), err.message));
      parfor_numworkers = 0; % Kein parfor benutzen
      continue % Nochmal neu versuchen oder Ende der Funktion ohne ParPool
    end
  else
    parfor_numworkers = Pool.NumWorkers;
  end
  if ~isinf(Set.general.parcomp_maxworkers) && parfor_numworkers ~= Set.general.parcomp_maxworkers
    cds_log(-1, sprintf(['Die gewünschte Zahl von %d Parallelinstanzen ', ...
      'konnte nicht erfüllt werden. Es sind jetzt %d.'], ...
      Set.general.parcomp_maxworkers, parfor_numworkers));
  else
    cds_log(1, sprintf(['[start_parpool] ParPool wurde gestartet. ', ...
      'Dauer: %1.1fs. %d parallele Instanzen'], toc(t0), parfor_numworkers));
  end
  % Warnungen auch in ParPool-Workern unterdrücken
  if parfor_numworkers > 1 % nur, wenn Pool-Start oben erfolgreich war
    try
      parfevalOnAll(gcp(), @warning, 0, 'off', 'MATLAB:singularMatrix');
      parfevalOnAll(gcp(), @warning, 0, 'off', 'MATLAB:nearlySingularMatrix');
      parfevalOnAll(gcp(), @warning, 0, 'off', 'MATLAB:illConditionedMatrix');
      parfevalOnAll(gcp(), @warning, 0, 'off', 'Coder:MATLAB:rankDeficientMatrix');
      parfevalOnAll(gcp(), @warning, 0, 'off', 'Coder:MATLAB:nearlySingularMatrix');
      parfevalOnAll(gcp(), @warning, 0, 'off', 'Coder:MATLAB:illConditionedMatrix');
      % Siehe https://github.com/altmany/export_fig/issues/75
      parfevalOnAll(gcp(), @warning, 0, 'off', 'MATLAB:prnRenderer:opengl');
    catch err
      cds_log(-1, sprintf('[start_parpool] Fehler beim Konfigurieren des parpool: %s.', err.message));
      continue % Nochmal neu versuchen oder Ende der Funktion ohne diese Konfiguration
    end
  end
  break; % Bis hier gekommen. Also erfolgreich gestartet.
end
if Set.general.isoncluster
  parpool_writelock('free', 0, true);
end