% Start der komb. Struktur und Maßsynthese für alle Roboter
% Erwartet Workspace-Variablen:
% Set (Globale Einstellungen)
% Traj (Eigenschaften der Trajektorie)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

if ~exist('Set', 'var') || ~exist('Traj', 'var')
  error('Eingabevariablen des Startskriptes existieren nicht');
end
mkdirs(fullfile(Set.optimization.resdir, Set.optimization.optname));


% Menge der Roboter laden
Structures = cds_gen_robot_list(Set.structures);

% Optimierung der Strukturen durchführen
for i = 7%:length(Structures)
  % Maßsynthese für diesen Roboter starten
  fprintf('Starte Maßsynthese für Roboter %d (%s)\n', i, Structures{i}.Name);
  RobotOptRes = cds_dimsynth_robot(Set, Traj, Structures{i});
  % Ergebnisse speichern
  save(fullfile(Set.optimization.resdir, Set.optimization.optname, ...
    sprintf('Rob%d_%s', i, Structures{i}.Name)), 'RobotOptRes', 'Set', 'Traj');
end

% Ergebnisse vergleichen

% Ergebnisse darstellen
cds_vis_results(Set, Traj, Structures);