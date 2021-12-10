% Kombinierte Struktur- und Maßsynthese. Beispiel für Robotik-1-Vorlesung
% Optimiere eine Industrie-Roboter-Kinematik.
% Erzeugt ein Evolutions-Video für die Optimierung.
% 
% Damit dieses Beispiel funktioniert, müssen die Abhängigkeiten installiert
% werden, so wie in der README.MD dieses Repos beschrieben.
% Die Einstellungen zur Optimierung werden in cds_settings_defaults
% beschrieben.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

%% Einstellungen für anderen Programmfluss
% Schalter zum Nachverarbeiten der vom Cluster heruntergeladenen Ergebnisse
% (Zusammenfassen einer aufgeteilten Berechnung)
merge_results_only = false;
% Berechnung auf Cluster durchführen
cluster = false;

%% Einstellungen aus Aufgabengeometrie
% Aufgaben-FG (Rotationssymmetrische Aufgabe, Rotation um EE-z-Achse egal)
DoF = [1 1 1 1 1 0];
% Einstellungs-Struktur initialisieren
Set = cds_settings_defaults(struct('DoF', DoF));

%% Bauraum
% Der Roboter soll innerhalb eines stehenden Zylinders arbeiten.
% Höhe 3m, Radius 2m.
Set.task.installspace = struct( ...
  'type', uint8(2), ... % Zylinder
  'params', [[0, 0, 0], [0, 0, 2], 1], ... 
  'links', {{0:6}}); % Alle Gelenke müssen in dem Zylinder sein

%% Eigenschaften des Roboters, von Aufgabengeometrie beeinflusst
% Der Roboter soll eher mittig im Arbeitsraum sein
Set.optimization.basepos_limits = [[-0.8, 0.8]; [-0.8, 0.8]; [0 1]];

% Kollisionsprüfung ist notwendig.
Set.optimization.constraint_collisions = true;
% Annahme: Kein zusätzlicher Endeffektor
Set.optimization.ee_rotation = true;
Set.optimization.ee_translation = true;

%% Zusätzliche Masse am Endeffektor
% Repräsentiert das Werkzeug. Annahme: 5kg
Set.task.payload = struct('m', 5, 'rS', zeros(3,1), 'Ic', zeros(6,1));
Set.task.payload.Ic(1:3) =  2/5 * Set.task.payload.m * (60e-3)^2; % Kugel Radius 60mm

%% Trajektorie laden
Set.task.maxangle = 30*pi/180;
% Beispiel-Trajektorie in die Mitte des Arbeitsraums setzen
Traj = cds_gen_traj(DoF, 3, Set.task);
Traj.X(:,3) = Traj.X(:,3) + 1.0;
Traj.XE(:,3) = Traj.XE(:,3) + 1.0;
% Debug: Visualisierung der Aufgabe
% cds_show_task(Traj, Set)

%% Sonstige Optimierungseinstellungen
optname = 'indrob_example_robotik1';
num_repetitions = 1; % Anzahl der Wiederholungen der Optimierung
Set.optimization.NumIndividuals = 30;
Set.optimization.MaxIter = 50;
Set.general.plot_details_in_fitness = 0*4e9; % debug:
Set.general.plot_robot_in_fitness = 1e3;
Set.general.verbosity = 4;
Set.general.matfile_verbosity = 3;
Set.general.save_robot_details_plot_fitness_file_extensions = {'png'};
Set.general.animation_styles = {'3D'};
Set.general.save_animation_file_extensions = {'mp4'};
Set.general.maxduration_animation = 10;  % Länge begrenzen
Set.general.eval_figures = {'pareto_all_phys', 'robvisuanim'};
% Debug: Auswahl der Strukturen
Set.structures.whitelist = {'S6RRRRRR10V2'}; % Industrieroboter-Kinematik

% Für den Fall, dass keine Positiv-Liste genommen wird: Strukturen filtern
Set.structures.joint_filter = 'RRRRRR'; % Nur Drehgelenke
Set.structures.use_serial = true;
Set.structures.use_parallel = false;

% Nehme antriebsredundante 3T3R-Roboter, falls Aufgabe als 3T2R definiert.
Set.structures.max_task_redundancy = 1;

% Konditionszahl darf nicht total schlecht sein. Dann werden
% Antriebskräfte und innere Kräfte zu hoch (Erfahrungswerte)
Set.optimization.constraint_obj(4) = 500; % max. Wert für Konditionszahl
% Die Antriebskraft sollte nicht zu groß werden.
Set.optimization.constraint_obj(3) = 1000; % max. Wert in Nm

% Debug: Abbruchkriterien so definieren, dass nur eine einzige gültige
% Lösung direkt genommen wird
% Set.optimization.obj_limit = [1e3;1e3]; % Unter 1e3 ist gültig.

if cluster
  Set.general.parcomp_struct = true; % Maßsynthese parallel durchführen
  Set.general.parcomp_plot = true; % Bilder parallel erzeugen
  Set.general.computing_cluster = true; % Auf Cluster rechnen
  Set.general.computing_cluster_max_time = 3600; % eine Stunde
end
% Debug: Bei Abbruch der Berechnung Ergebnis erzeugen
% Set.general.only_finish_aborted = true;
% Set.general.regenerate_summary_only = true;
Set.general.save_evolution_video = true;

% Debug: Mehrere parallele Durchläufe des gleichen Roboters
% Set.structures.repeatlist = {{Set.structures.whitelist{1},8}};

% Keine alten Ergebnisse nehmen. Sonst tritt nicht der gewünschte Effekt
% auf, dass die Roboter im Evolutionsvideo immer besser werden.
Set.optimization.InitPopRatioOldResults = 0;
%% Starten
for ps = 1 % Pareto-Kombinationen
  if ps == 1
    Set.optimization.objective = {'actforce', 'chainlength'};
  else
    error('Fall %d nicht definiert', ps);
  end
  for k = 1:num_repetitions
    if num_repetitions == 1 % Ohne Wiederholung
      Set.optimization.optname = sprintf('%s_ps%d', optname, ps);
    else % Mehrfache Durchführung
      Set.optimization.optname = sprintf('%s_ps%d_rep%d', optname, ps, k);
    end
    if ~merge_results_only
      cds_start(Set, Traj);
      pause(30); % Damit nicht alle exakt zeitgleich starten; paralleler Start des parpools nicht möglich
    else
      cds_merge_results( Set.optimization.optname, 'copy', true, true );
    end
  end
end