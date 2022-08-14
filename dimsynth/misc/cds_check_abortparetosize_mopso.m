% OutputFcn für den Multi-Objective Particle-Swarm-Algorithmus (MOPSO). 
% Prüfe die Abbruchbedingung aufgrund der Größe der Pareto-Front
% 
% Die Funktion kann mit folgendem Befehl im MOPSO-Algorithmus benutzt werden:
% mopso_outputfun = @(MS)cds_check_abortparetosize_mopso(MS,Set,Structure);
% options.OutputFcn = {mopso_outputfun};
% 
% Eingabe:
% MOPSO_struct
%   Standard-Eingabe für MOPSO-Output-Fcn. Siehe MOPSO.m, OutputFcn
%   Felder: POS, POS_fit, REP
% Set
%   Eingabe zusätzlich zu MOPSO-Standard.
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% 
% Ausgabe:
% stop
%   Standard-Ausgabe für MOPSO-Output-Fcn

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-06
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function stop = cds_check_abortparetosize_mopso(MOPSO_struct, Set)
stop = false;
REP = MOPSO_struct.REP;
if isempty(REP)
  return
end
if size(REP.pos,1) > Set.optimization.abort_pareto_front_size
  stop = true;
  cds_log(1,sprintf('[output] Pareto-Front enthält %d Einträge. Abbruch.', size(REP.pos,1)));
end
