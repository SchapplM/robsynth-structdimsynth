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
warning('off', 'MATLAB:rankDeficientMatrix');
warning('off', 'Coder:MATLAB:singularMatrix');
warning('off', 'Coder:MATLAB:nearlySingularMatrix');
warning('off', 'Coder:MATLAB:rankDeficientMatrix');
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
  parfevalOnAll(gcp(), @warning, 0, 'off', 'Coder:MATLAB:nearlySingularMatrix');
  parfevalOnAll(gcp(), @warning, 0, 'off', 'Coder:MATLAB:illConditionedMatrix');
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

if ~isempty(Set.structures.whitelist)
  if length(Set.structures.whitelist) ~= length(unique(Set.structures.whitelist))
    error('Die Positiv-Liste enthält doppelte Einträge');
  end
  Names_in_Struct = {}; % Es können bei Struktursynthese Strukturen doppelt getestet werden
  for i = 1:length(Structures), Names_in_Struct{i} = Structures{i}.Name; end %#ok<SAGROW>
  if length(Set.structures.whitelist) ~= length(unique(Names_in_Struct))
    warning('Es wurde eine Positiv-Liste übergeben, aber nicht alle dieser Strukturen wurden gewählt.');
  end
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
  % Vorbereitung: Getrennt für serielle und parallele Roboter
  for type = [0 2] % seriell und PKM
    % Stelle vorher eine Liste von Namen zusammen, um doppelte zu finden.
    Names = {};
    for k = 1:length(Structures)
      if Structures{k}.Type == type
        Names = [Names, Structures{k}.Name]; %#ok<AGROW>
      end
    end
    % Duplikate löschen (treten z.B. auf, wenn verschiedene Werte für theta
    % in der Struktursynthese möglich sind)
    Names = unique(Names);
    % Vorlagen-Funktionen neu generieren (falls dort Änderungen gemacht
    % wurden). Die automatische Neugenerierung in der parfor-Schleife
    % funktioniert nicht aufgrund von Dateikonflikten, autom. Ordnerlöschung.
    if type == 2 % Sperrschutz für PKM-Bibliothek (hauptsächlich für Struktursynthese)
      parroblib_writelock('lock', 'mex', logical(Set.structures.DoF)); % keine gleichzeitige Änderung erlauben.
    end
    if Set.general.create_template_functions
      for i = 1:length(Names)
        if type == 0 % Serieller Roboter
          serroblib_create_template_functions(Names(i), false, false);
        else % PKM
          parroblib_create_template_functions(Names(i), false, false);
        end
        continue
      end
    end
    if Set.general.use_mex && Set.general.compile_missing_functions
      % Benötigte Funktionen kompilieren (serielle statt parallele Ausführung)
      % (es wird automatisch der codegen-Ordner gelöscht. Kann bei paralleler
      % Rechnung zu Konflikten führen)
      t1 = tic(); % Beginn der Prüfung auf Datei-Existenz
      t_ll = t1; % Zeitpunkt der letzten Log-Ausgabe diesbezüglich
      for i = 1:length(Names)
        if type == 0 % Serieller Roboter
          R = serroblib_create_robot_class(Names{i});
        else % PKM
          R = parroblib_create_robot_class(Names{i},1,1);
        end
        % Hierdurch werden fehlende mex-Funktionen kompiliert.
        R.fill_fcn_handles(true, true);
        if toc(t_ll) > 20
          fprintf('%d/%d Roboter vom Typ %d auf Existenz der Dateien geprüft. Dauer bis hier: %1.1fs\n', ...
            i, length(Names), type, toc(t1));
          t_ll = tic();
        end
      end
    end
    if type == 2 % Sperrschutz für PKM-Repo aufheben
      parroblib_writelock('free', 'mex', logical(Set.structures.DoF));
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
