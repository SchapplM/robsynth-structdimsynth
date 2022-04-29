% Initialisiere den Matlab-ParPool mit Einstellungen aus der Maßsynthese
% 
% Eingabe:
% Set
%   Struktur aus Maßsynthese. Siehe cds_start und cds_settings_defaults
% 
% Ausgabe:
% parfor_numworkers
%   Anzahl der tatsächlich gestarteten ParPool-Worker

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-04
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function parfor_numworkers = cds_start_parpool(Set)
if Set.general.isoncluster % auf Cluster möglicher Zugriffskonflikt für ParPool
  parpool_writelock('lock', 180, true); % Synchronisationsmittel für ParPool
end
Pool = gcp('nocreate');
if isempty(Pool)
  try
    cds_log(1, sprintf('[start_parpool] Starte ParPool mit Ziel parfor_numworkers=%d', ...
      Set.general.parcomp_maxworkers));
    Pool=parpool([1,Set.general.parcomp_maxworkers]);
    parfor_numworkers = Pool.NumWorkers;
  catch err
    cds_log(1, sprintf('[start_parpool] Fehler beim Starten des parpool: %s', err.message));
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
  cds_log(-1, sprintf(['Die gewünschte Zahl von %d Parallelinstanzen ', ...
    'konnte nicht erfüllt werden. Es sind jetzt %d.'], ...
    Set.general.parcomp_maxworkers, parfor_numworkers));
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