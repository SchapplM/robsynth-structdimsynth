% OutputFcn für den Particle-Swarm-Algorithmus. Speichere den aktuellen
% Zwischenstand.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function stop = cds_psw_save_all_results(optimValues,state, Set, Structure)

stop = false;
resdir = fullfile(Set.optimization.resdir, Set.optimization.optname, ...
  'tmp', sprintf('%d_%s', Structure.Number, Structure.Name));
if strcmp(state, 'init') && exist(resdir, 'file')
  % Leere Verzeichnis
  rmdir(resdir, 's')
end
mkdirs(resdir);
filename = sprintf('PSO_Gen%02d_AllInd_%s.mat', optimValues.iteration, state);
save(fullfile(resdir, filename), 'optimValues');
fprintf('Zwischenergebnisse gespeichert: %s\n', fullfile(resdir, filename));