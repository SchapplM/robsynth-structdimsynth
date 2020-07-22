% Start der komb. Struktur und Maßsynthese für alle Roboter
% Erwartet Workspace-Variablen:
% Set (Globale Einstellungen)
% Traj (Eigenschaften der Trajektorie)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

% Warnungen unterdrücken, die bei der Maßsynthese typischerweise auftreten
warning('off', 'MATLAB:singularMatrix');
warning('off', 'MATLAB:nearlySingularMatrix');
warning('off', 'MATLAB:illConditionedMatrix');
warning('off', 'Coder:MATLAB:rankDeficientMatrix');

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

% Bei paralleler Berechnung dürfen keine Dateien geschrieben werden um
% Konflikte zu vermeiden
if Set.general.parcomp_struct && ... % Parallele Rechnung ist ausgewählt
    ~Set.general.regenerate_summmary_only && ... % für Bildgenerierung ParComp nicht benötigt
    length(Structures) > 1 % für Optimierung eines Roboters keine parallele Rechnung
  % Keine Bilder zeichnen
  Set.general.plot_details_in_fitness = 0;
  Set.general.plot_robot_in_fitness = 0;
  Set.general.noprogressfigure = true;
  % Keine (allgemeinen) mat-Dateien speichern
  Set.general.matfile_verbosity = 0;
  try %#ok<TRYNC>
    parpool(Set.general.parcomp_maxworkers);
  end
  Pool=gcp();
  parfor_numworkers = Pool.NumWorkers;
  if ~isinf(Set.general.parcomp_maxworkers) && parfor_numworkers ~= Set.general.parcomp_maxworkers
    warning('Die gewünschte Zahl von %d Parallelinstanzen konnte nicht erfüllt werden. Es sind jetzt %d.', ...
      Set.general.parcomp_maxworkers, parfor_numworkers)
  end
  % Warnungen auch in ParPool-Workern unterdrücken
  parfevalOnAll(gcp(), @warning, 0, 'off', 'MATLAB:singularMatrix');
  parfevalOnAll(gcp(), @warning, 0, 'off', 'MATLAB:nearlySingularMatrix');
  parfevalOnAll(gcp(), @warning, 0, 'off', 'MATLAB:illConditionedMatrix');
  parfevalOnAll(gcp(), @warning, 0, 'off', 'Coder:MATLAB:rankDeficientMatrix');
else
  parfor_numworkers = 0;
end

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
    && ~any(strcmp(Set.optimization.objective, {'mass', 'energy', 'stiffness'})) ...
    && ~any(Set.optimization.constraint_obj([1 5]))
  Set.optimization.use_desopt = false;
  fprintf(['Entwurfsoptimierung wurde verlangt, aber keine dafür notwendigen ', ...
    'Zielfunktionen oder Nebenbedingungen definiert. Wurde wieder deaktiviert\n']);
end
% Optimierung der Strukturen durchführen
if ~Set.general.regenerate_summmary_only
  % Vorlagen-Funktionen neu generieren (falls dort Änderungen gemacht
  % wurden). Die automatische Neugenerierung in der parfor-Schleife
  % funktioniert nicht.
  if Set.general.create_template_functions
    for i = 1:length(Structures)
      Structure = Structures{i};
      if Structure.Type == 1 % Serieller Roboter
        serroblib_create_template_functions({Structure.Name}, false, false);
      else % PKM
        parroblib_create_template_functions({Structure.Name}, false, false);
      end
      continue
    end
  end
  
  resdir_main = fullfile(Set.optimization.resdir, Set.optimization.optname);
  mkdirs(resdir_main); % Ergebnis-Ordner für diese Optimierung erstellen
  t1 = tic();
  parfor (i = 1:length(Structures), parfor_numworkers)
    % Maßsynthese für diesen Roboter starten
    fprintf('Starte Maßsynthese für Roboter %d (%s)\n', i, Structures{i}.Name);
    cds_dimsynth_robot(Set, Traj, Structures{i});
  end
  fprintf('Optimierung von %d Robotern abgeschlossen. Dauer: %1.1fs\n', length(Structures), toc(t1));
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
