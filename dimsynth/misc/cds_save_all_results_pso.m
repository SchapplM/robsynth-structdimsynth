% OutputFcn für den Particle-Swarm-Algorithmus. Speichere den aktuellen
% Zwischenstand.
% 
% Die Funktion kann mit folgendem Befehl im PSO-Algorithmus benutzt werden:
% pso_outputfun = @(optimValues,state)cds_save_all_results_pso(optimValues,state,Set,Structure);
% options.OutputFcn = {pso_outputfun};
% 
% Eingabe:
% optimValues
%   Standard-Eingabe für PSO-Output-Fcn
% state
%   Standard-Eingabe für PSO-Output-Fcn
% Set
%   Eingabe zusätzlich zu PSO-Standard.
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Structure
%   Eingabe zusätzlich zu PSO-Standard.
%   Eigenschaften der Roboterstruktur (aus cds_gen_robot_list.m)
% 
% Ausgabe:
% stop
%   Standard-Ausgabe für PSO-Output-Fcn

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function stop = cds_save_all_results_pso(optimValues, state, Set, Structure)

stop = false;
if ~strcmp(state, 'iter')
  return
end
% Detail-Ergebnisse extrahieren (persistente Variable in Funktion)
PSO_Detail_Data = cds_save_particle_details(Set, [], 0, 0, NaN, NaN, NaN, NaN, 'output');

% Aktuellen Stand der Optimierung speichern
resdir = fullfile(Set.optimization.resdir, Set.optimization.optname, ...
  'tmp', sprintf('%d_%s', Structure.Number, Structure.Name));
filename = sprintf('PSO_Gen%02d_AllInd.mat', optimValues.iteration);
save(fullfile(resdir, filename), 'optimValues', 'PSO_Detail_Data');
% Datei der vorherigen Iteration löschen (wird nicht mehr benötigt)
if optimValues.iteration > 1
  filename_previous = sprintf('PSO_Gen%02d_AllInd.mat', optimValues.iteration-1);
  delete(fullfile(resdir, filename_previous));
end

cds_log(1,sprintf('[output] Zwischenergebnisse gespeichert: %s', ...
  fullfile(resdir, filename)));