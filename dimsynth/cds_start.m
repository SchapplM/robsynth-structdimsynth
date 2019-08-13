% Start der komb. Struktur und Maßsynthese für alle Roboter
% Erwartet Workspace-Variablen:
% Set (Globale Einstellungen)
% Traj (Eigenschaften der Trajektorie)

if ~exist('Set', 'var') || ~exist('Traj', 'var')
  error('Eingabevariablen des Startskriptes existieren nicht');
end

% Menge der Roboter laden
Structures = cds_gen_robot_list(Set.structures);

