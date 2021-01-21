% OutputFcn für den Multi-Objective GA. Speichere den aktuellen
% Zwischenstand.
% 
% Die Funktion kann mit folgendem Befehl im gamultiobj-Algorithmus benutzt werden:
% gamo_outputfun = @(options,state,flag)cds_save_all_results_gamultiobj(options,state,flag,Set,Structure);
% options.OutputFcn = {gamo_outputfun};
% 
% Eingabe:
% options, state, flag
%   Standard-Eingabe für GAMULTIOBJ-Output-Fcn. Siehe gaoutputfcntemplate.m
% Set
%   Eingabe zusätzlich zu GAMULTIOBJ-Standard.
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Structure
%   Eingabe zusätzlich zu GAMULTIOBJ-Standard.
%   Eigenschaften der Roboterstruktur (aus cds_gen_robot_list.m)
% 
% Ausgabe:
% state,options,optchanged
%   Standard-Ausgabe für GAMULTIOBJ-Output-Fcn

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [state,options,optchanged] = cds_save_all_results_gamultiobj(options, state, flag, Set, Structure)

optchanged = false;
if any(isinf(state.Score(:)))
  % Der Wert inf wird in cds_fitness als Marker benutzt, um die Optimierung
  % abzubrechen. Das kann hier an den MOGA übergeben werden, falls die
  % Abbruchbedingung nicht in den MOGA-Optionen selbst definiert ist.
  state.stopflag = 'y';
end
if ~strcmp(flag, 'iter')
  return
end
% Detail-Ergebnisse extrahieren (persistente Variable in Funktion)
PSO_Detail_Data = cds_save_particle_details(Set, [], 0, 0, NaN, NaN, NaN, NaN, 'output');

% Aktuellen Stand der Optimierung speichern
resdir = fullfile(Set.optimization.resdir, Set.optimization.optname, ...
  'tmp', sprintf('%d_%s', Structure.Number, Structure.Name));
filename = sprintf('GAMO_Gen%02d_AllInd.mat', state.Generation);
save(fullfile(resdir, filename), 'state', 'PSO_Detail_Data');
% Datei der vorherigen Iteration löschen (wird nicht mehr benötigt)
if state.Generation > 1
  filename_previous = sprintf('GAMO_Gen%02d_AllInd.mat', state.Generation-1);
  delete(fullfile(resdir, filename_previous));
end

cds_log(1,sprintf('[output] Zwischenergebnisse gespeichert: %s', ...
  fullfile(resdir, filename)));