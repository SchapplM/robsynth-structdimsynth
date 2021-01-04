% OutputFcn für den Multi-Objective Particle-Swarm-Algorithmus (MOPSO). 
% Speichere den aktuellen Zwischenstand.
% 
% Die Funktion kann mit folgendem Befehl im MOPSO-Algorithmus benutzt werden:
% mopso_outputfun = @(REP)cds_save_all_results_mopso(REP,Set,Structure);
% options.OutputFcn = {mopso_outputfun};
% 
% Eingabe:
% REP
%   Standard-Eingabe für MOPSO-Output-Fcn
% Set
%   Eingabe zusätzlich zu MOPSO-Standard.
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Structure
%   Eingabe zusätzlich zu MOPSO-Standard.
%   Eigenschaften der Roboterstruktur (aus cds_gen_robot_list.m)
% 
% Ausgabe:
% stop
%   Standard-Ausgabe für MOPSO-Output-Fcn

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function stop = cds_save_all_results_mopso(REP, Set, Structure)

stop = false;
% Detail-Ergebnisse extrahieren (persistente Variable in Funktion)
PSO_Detail_Data = cds_save_particle_details(Set, [], 0, 0, NaN, NaN, NaN, NaN, 'output');
% Nummer der aktuellen Generation herausfinden
currgen = find(any(~isnan(PSO_Detail_Data.comptime),2),1,'last')-1;
filename = sprintf('MOPSO_Gen%02d_AllInd.mat', currgen);

% Aktuelle Pareto-Front und Details abspeichern
resdir = fullfile(Set.optimization.resdir, Set.optimization.optname, ...
  'tmp', sprintf('%d_%s', Structure.Number, Structure.Name));
save(fullfile(resdir, filename), 'PSO_Detail_Data', 'REP');
% Datei der vorherigen Iteration löschen (wird nicht mehr benötigt)
if currgen > 1
  filename_previous = sprintf('MOPSO_Gen%02d_AllInd.mat', currgen-1);
  delete(fullfile(resdir, filename_previous));
end

cds_log(1,sprintf('[output] Zwischenergebnisse gespeichert: %s', ...
  fullfile(resdir, filename)));