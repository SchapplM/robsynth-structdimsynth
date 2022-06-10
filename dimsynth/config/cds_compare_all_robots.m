% Führe die kombinierte Struktur- und Maßsynthese für alle Roboter durch.
% Damit werden Videos erzeugt, mit denen man die Wirksamkeit zeigen kann.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-11
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc

% Benutzereinstellungen
usr_cluster = true;
usr_only_pkm = false;
usr_only_serial = false;


% Definition aller FG
EEFG_Ges = logical(...
  [1 1 0 0 0 1; ...
   1 1 1 0 0 0; ...
   1 1 1 0 0 1; ...
   1 1 1 1 1 0; ...
   1 1 1 1 1 1]);
%% Optimierung aller Roboter für alle Aufgabentypen starten
for i_FG = 1:size(EEFG_Ges,1) % Alle FG einmal durchgehen
  for i_AR = 0:1 % mit und ohne Aufgabenredundanz
    EEFG = EEFG_Ges(i_FG,:);
    if i_AR == 1 && (all(EEFG==[1 1 1 0 0 0]) || all(EEFG==[1 1 1 1 1 0]))
      continue % Bei 3T0R und 3T2R keine Aufgabenredundanz definiert
    end
    if i_AR == 1 % Aufgabenredundanz
      EEFG(end) = 0;
    end
    % Einstellungen der Optimierung
    Set = cds_settings_defaults(struct('DoF', EEFG));
    if i_FG == 1
      trajno = 2;
    elseif i_FG < 4
      trajno = 1;
    else
      trajno = 3;
    end
    if i_FG == 3
      % Bei 3T1R-PKM können keine großen Rotationen erreicht werden.
      % Begrenze diese sehr stark gegenüber der Standardeinstellung
      Set.task.maxangle = 5*pi/180;
    end
    % Überschreiben der Trajektorien-Einstellung. Nur Bewegung in jede
    % Richtung einmal machen.
%     trajno = 4;
    if i_AR == 1 % Aufgabenredundanz
      Set.task.pointing_task = true;
      Set.structures.min_task_redundancy = 1; % Fordere Aufgabenredundanz
      Set.structures.max_task_redundancy = 1; % Erlaube Aufgabenredundanz
    end
    Traj = cds_gen_traj(EEFG, trajno, Set.task);
  %   continue
    % Verschiebe die Aufgabe
    task_dim = (max(Traj.XE(:,1:3)) - min(Traj.XE(:,1:3)));
    task_mid1 = min(Traj.XE(:,1:3)) + task_dim/2;
    delta_x_task = -task_mid1; % Erst auf Ursprung ziehen
    if ~all(EEFG==[1 1 0 0 0 1]) % Bei 2T1R muss die z-Position der PKM identisch zur Basis-Position sein
      delta_x_task = delta_x_task + [0,0,0.4]; % höher ziehen (quasi auf Tisch)
    end
    Traj.XE(:,1:3) = Traj.XE(:,1:3) + repmat(delta_x_task, size(Traj.XE,1),1);
    Traj.X(:,1:3) = Traj.X(:,1:3) + repmat(delta_x_task, size(Traj.X,1),1);
    task_mid = min(Traj.XE(:,1:3)) + task_dim/2;
    % moderate Anzahl an Wiederholungen (reicht für gute Konvergenz)
    Set.optimization.NumIndividuals = 100;
    Set.optimization.MaxIter = 50;
    % Debug: Bei erstem funktionierendem Ergebnis aufhören
    Set.optimization.obj_limit = 1e3;
    % Debug: Bei Selbstkollision aufhären
%     Set.optimization.obj_limit = 4e9;
    % Debug: Beschränkung auf seriell oder parallel
    if usr_only_pkm
      Set.structures.use_serial = false;
    end
    if usr_only_serial
      Set.structures.use_parallel = false;
    end
    % Parallel und auf Cluster
    Set.general.computing_cluster = usr_cluster; % Auf Cluster rechnen
    Set.general.computing_cluster_cores = 8;
    Set.general.cluster_maxrobotspernode = Set.general.computing_cluster_cores;
    Set.general.computing_cluster_max_time = 4*3600; % max 4h pro Roboter

    % Text- und Bildausgabe definieren
    Set.general.verbosity = 3;
    % Debug-Bilder (werden auf Cluster nicht erzeugt)
%     Set.general.plot_details_in_fitness = 1e11;
%     Set.general.plot_robot_in_fitness = 1e11;
    % Bilder nicht speichern
    Set.general.save_robot_details_plot_fitness_file_extensions = {};
    % Für jeden Roboter eine Animation erstellen
    Set.general.animation_styles = {'3D'};
    Set.general.save_animation_file_extensions = {'mp4'};
    Set.general.eval_figures = {}; % Keine Einzelauswertungs-Bilder
    Set.general.noprogressfigure = true;
    % Debug: Für schnellere Iterationen ohne Änderung der Funktionen
%     Set.general.create_template_functions = false;
%     Set.general.update_template_functions = false;

    Set.optimization.objective = {'mass'}; % Nehme den leichtesten Roboter (am kürzesten)
    Set.optimization.constraint_obj(4) = 1000*100; % max. Wert für Konditionszahl
    % Keine Selbstkollisionen (sieht sonst unplausibel im Video aus)
    Set.optimization.constraint_collisions = true;
    % Begrenzung der Aufgabe
    Set.optimization.base_size_limits = [0.1, 1.5]; % PKM-Basis nicht viel zu groß
    Set.optimization.base_morphology = true;
    Set.optimization.platform_morphology = true;
    Set.optimization.rotate_base = true;
    Set.optimization.ee_rotation = false;
    Set.optimization.ee_translation_only_serial = true; % für PKM keine Verschiebung des EE
    
    % Benutze den Index aller Ergebnisse zum schnelleren Start
    if usr_cluster % Lokal immer alle Ordner durchsuchen für Aktualität und da lokal schneller
      Set.optimization.InitPopFromGlobalIndex = true;
    end
    % Definiere den Bauraum: Zylinder um die Aufgabe
    r_Zylinder = 2.0; % Radius in m
    h_Zylinder = 2; % Höhe in m
    p_Zylinder = [[0,0,0], ... % Erster Punkt (unten)
      [0,0,h_Zylinder], ... % Zweiter Punkt (oben)
      r_Zylinder, NaN(1,3)];
    Set.task.installspace = struct( ...
      'type', uint8(2), ... % Nummern der Geometrie-Typen, siehe Implementierung.
      'params', p_Zylinder, ...  % (2) Quader
      'links', {{0:6}}); % Alle Teile des Roboters müssen im Bauraum sein
    
    % Definiere eine Kugel in der unteren Hälfte des Arbeitsraums als Grenze
    % Weglassen für mehr i.O.-Ergebnisse
  %   r = mean(max(Traj.XE(:,1:2)) - min(Traj.XE(:,1:2)))/2;
  %   % Höhe des Mittelpunkts der Kugel
  %   h = min(Traj.XE(:,3)) -r - 0.5*max(Traj.XE(:,3)) - min(Traj.XE(:,3));
  %   if i_FG > 1 % nur bei xyz kartesischen FG sinnvoll
  %     Set.task.obstacles = struct( ...
  %       'type', uint8(4), ... % Kugel
  %       'params', [mean(minmax2(Traj.XE(:,1:3)')')+[0,0,h], r]); % Mittelpunkt, Radius
  %   end
    % Debug: Visualisierung der Aufgabe
  %   cds_show_task(Traj, Set)
    % Debug: Nur einen Roboter erzeugen. Wird manuell eingetragen, wenn
    % Ergebnisse genauer lokal untersucht werden sollen.
    if i_FG == 1
%       Set.structures.whitelist = {'P3RRR1G1P1A1'};
    elseif i_FG == 2
%       Set.structures.whitelist = {'P3PRRRR6V1G3P2A1'};
    elseif i_FG == 3
%       Set.structures.whitelist = {'S4RPRR1'};
    elseif i_FG == 4
%       Set.structures.whitelist = {'S5RRRPR2'};
    elseif i_FG == 5
%       Set.structures.whitelist = {'P6RRRRRR10G1P4A2'};
    end
    if Set.general.computing_cluster && length(Set.structures.whitelist) == 1
      warning('Es soll nur ein Roboter auf dem Cluster berechnet werden. Sicher?');
      pause(5); % Bedenkzeit
    end
    EEFG_Rob = EEFG;
    if i_AR == 1 % Aufgabenredundanz
      trstr = '_taskred';
      EEFG_Rob(end) = 1;
    else
      trstr = '';
    end
    if usr_only_pkm
      filtstr = '_pkm';
    elseif usr_only_serial
      filtstr = '_serial';
    else
      filtstr = '';
    end
    Set.optimization.optname = sprintf('allrobots_%dT%dR_%s%s', ... % bei erneutem Start: z.B. Datum einfügen
      sum(EEFG_Rob(1:3)), sum(EEFG_Rob(4:6)), trstr, filtstr);

    % Debug: Falls Berechnung abgebrochen ist und Videos nachträglich erstellt werden sollen
  %   Set.general.regenerate_summary_only = true;
    cds_start(Set, Traj);
  end
end
