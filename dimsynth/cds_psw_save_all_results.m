% OutputFcn für den Particle-Swarm-Algorithmus. Speichere den aktuellen
% Zwischenstand.
% 
% Die Funktion kann mit folgendem Befehl im PSO-Algorithmus benutzt werden:
% cds_save_all_results_anonym = @(optimValues,state)cds_psw_save_all_results(optimValues,state,Set,Structure);
% options.OutputFcn = {cds_save_all_results_anonym};
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
% (C) Institut für Mechatronische Systeme, Universität Hannover

function stop = cds_psw_save_all_results(optimValues,state, Set, Structure)

stop = false;
resdir = fullfile(Set.optimization.resdir, Set.optimization.optname, ...
  'tmp', sprintf('%d_%s', Structure.Number, Structure.Name));
filename = sprintf('PSO_Gen%02d_AllInd_%s.mat', optimValues.iteration, state);
save(fullfile(resdir, filename), 'optimValues');
cds_log(1,sprintf('[output] Zwischenergebnisse gespeichert: %s', fullfile(resdir, filename)));