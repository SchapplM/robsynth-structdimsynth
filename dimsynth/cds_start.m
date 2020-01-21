% Start der komb. Struktur und Maßsynthese für alle Roboter
% Erwartet Workspace-Variablen:
% Set (Globale Einstellungen)
% Traj (Eigenschaften der Trajektorie)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

% Warnungen unterdrücken, die bei der Maßsynthese typischerweise auftreten
warning('off', 'MATLAB:singularMatrix');
warning('off', 'MATLAB:nearlySingularMatrix');

% Globale Variable zum Speichern von Ergebnis-Details der Fitness-Funktion
global PSO_Detail_Data

if ~exist('Set', 'var') || ~exist('Traj', 'var')
  error('Eingabevariablen des Startskriptes existieren nicht');
end

% Eingabe prüfen
Set_default = cds_settings_defaults(struct('DoF', Set.structures.DoF));
for subconf = fields(Set_default)'
  for ftmp = fields(Set.(subconf{1}))'
    if ~isfield(Set_default.(subconf{1}), ftmp{1})
      warning('Feld %s in der Eingabestruktur ist nicht vorgesehen', ftmp{1})
    end
  end
end
if Set.task.profile == 0 && strcmp(Set.optimization.objective, 'energy')
  error('Energieberechnung ohne Zeitverlauf der Trajektorie nicht sinnvoll');
end

% Menge der Roboter laden
Structures = cds_gen_robot_list(Set);

if isempty(Structures)
  fprintf('Keine Strukturen entsprechen den Filterkriterien\n');
  if ~isempty(Set.structures.whitelist)
    fprintf('Es wurde eine Positiv-Liste übergeben, aber keine Strukturen entsprachen den Kriterien. Filter-Liste passt nicht\n');
    return
  end
end
if ~isempty(Set.structures.whitelist) && length(Set.structures.whitelist) ~= length(Structures)
  warning('Es wurde eine Positiv-Liste übergeben, aber nicht alle dieser Strukturen wurden gewählt.');
end

if Set.optimization.use_desopt ... 
    && ~any(strcmp(Set.optimization.objective, {'mass', 'energy'})) ...
    && ~any(Set.optimization.constraint_obj(1))
  Set.optimization.use_desopt = false;
  fprintf(['Entwurfsoptimierung wurde verlangt, aber keine dafür notwendigen ', ...
    'Zielfunktionen oder Nebenbedingungen definiert. Wurde wieder deaktiviert\n']);
end

if ~Set.general.regenerate_summmary_only
% Optimierung der Strukturen durchführen
for i = 1:length(Structures)
  % Maßsynthese für diesen Roboter starten
  fprintf('Starte Maßsynthese für Roboter %d (%s)\n', i, Structures{i}.Name);
  % Globale Ergebnisvariable zurücksetzen
  Placeholder = NaN(Set.optimization.MaxIter+1, Set.optimization.NumIndividuals);
  PSO_Detail_Data = struct('comptime', Placeholder, 'fval', Placeholder);
  RobotOptRes = cds_dimsynth_robot(Set, Traj, Structures{i});
  % Ergebnisse speichern
  save(fullfile(Set.optimization.resdir, Set.optimization.optname, ...
    sprintf('Rob%d_%s_Endergebnis.mat', i, Structures{i}.Name)), ...
    'RobotOptRes', 'Set', 'Traj', 'PSO_Detail_Data');
end
end
if isempty(Structures)
  % Aufgrund der Filterkriterien wurden keine Roboter verglichen.
  % Auswertung ist nicht sinnvoll.
  return
end

% Ergebnisse darstellen
cds_results_table(Set, Traj, Structures)
cds_vis_results(Set, Traj, Structures);
cds_create_evolution_videos(Set, Traj, Structures)
