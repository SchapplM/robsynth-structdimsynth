% Standard-Einstellungen für kombinierte Struktur- und Maßsynthese
% 
% Eingabe:
% input_settings
%   Struktur mit Einstellungen zur Generierung aller
%   Optimierungseinstellungen
% 
% Ausgabe:
% settings
%   Struktur mit Feldern für alle Einstellungen der Optimierung
%   Die Einstellungen sind global und nicht spezifisch für den einzelnen
%   Roboter

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function settings = cds_settings_defaults(input_settings)
%% Standard-Eingabeeinstellungen setzen
input_settings_default = struct('DoF', logical([1 1 1 1 1 1]));
if nargin == 0
  input_settings = input_settings_default;
end
for f = fields(input_settings_default)'
  if ~isfield(input_settings, f{1})
    input_settings.(f{1}) = input_settings_default.(f{1});
  end
end
input_settings.DoF = logical(input_settings.DoF);
%% Allgemeine Einstellugen
% Einstellungen mit Auswirkungen auf vom Benutzer sichtbare Ergebnisse und
% deren Nachverarbeitung
general = struct( ...
  'verbosity', 2, ... % Ausgabeleven;0=minimal (Endergebnis), 1=mehr (Generationenweise), 2=noch mehr (Jede Fitness-Eval.), 3=alles, 4=Auch Entwurfsoptimierung
  'matfile_verbosity', 0, ... % Menge an mat-Dateien, die für Debug-Zwecke gespeichert werden; 0=keine, 1=wenige, 2=viele (erfolgreiche Ind.) 3=alle
  'plot_robot_in_fitness', 0, ... % Schwellwert der Gütefunktion zum Zeichnen von Details.
  'plot_details_in_fitness', 0, ... % Positiv: nur bei besseren; negativ: nur bei schlechteren als ...
  'plot_details_in_desopt' , 0, ... % Wie vorheriges Argument, aber für die Gütefunktion der Entwurfsoptimierung
  'save_robot_details_plot_fitness_file_extensions', {''}, ... % Speichern des durch vorherige Einstellung erstellten Bildes
  'save_animation_file_extensions', {{'gif', 'mp4'}}, ... % Format, in denen die Animationen gespeichert werden
  'animation_styles', {{'stick'}}, ... % Visualisierungsarten im Video: stick,3D,collision; bei mehr als einem: Syntax {{'1.','2.'}}
  'maxduration_animation', 10, ... % Die Animation soll max. 10s dauern (als Videodatei)
  'save_evolution_video', false, ... % Video mit Evolution der Roboter
  'max_retry_bestfitness_reconstruction', 2, ... % Anzahl Neuversuche zur Reproduktion. 2 reichen zur Prüfung, ob Wiederholbarkeit da ist.
  'regenerate_summmary_only', false, ... % Nur die Videos und Zusammenfassungsbilder neu generieren. Keine Optimierung durchführen.
  'nosummary', false, ... % Kompletter Verzicht auf die graphische Endauswertung
  'eval_figures', {{'histogram', 'fitness_various', 'jointtraj', 'robvisuanim', 'pareto_all_phys', 'pareto_all_fval'}}, ... % Liste der zu erstellenden Bilder. Auswahl, siehe cds_vis_results.m
  'only_save_summary_figures', true, ... % Bilderzeugung dient hauptsächlich der Speicherung. Schließe alle Bilder nach Speicherung
  'noprogressfigure', false, ... % Verzicht auf Fortschritts-Bild des PSO
  'debug_calc', false, ... % Doppelte Berechnung zur Prüfung von Funktionen
  'parcomp_struct', 0, ... % Parallele Berechnung unterschiedlicher Roboter. Enthält Anzahl der Worker-Instanzen als Zahl
  'parcomp_plot', 0, ... % Parallele Erzeugung der Ergebnisbilder und -videos
  'parcomp_maxworkers', inf, ... % Beschränkung der Anzahl paralleler Instanzen
  'computing_cluster', false, ... % Berechnung auf PBS-Cluster (Hochladen auf Server)
  'isoncluster', false, ... % Merker, ob gerade auf dem Cluster gerechnet wird. Dann sind einige Bilder und Debug-Auswertungen unnötig.
  'cluster_maxrobotspernode', inf, ... % Unbegrenzte Anzahl von Robotern auf jeder Cluster-Node. Niedrigere Zahl dient zur stärkeren Parallelisierung
  'compile_missing_functions', true, ... % Bei Start alle mex-Funktionen kompilieren
  'create_template_functions', false, ... % Erzeuge Funktionen neu aus Vorlagen-Dateien
  'use_mex', true);

%% Einstellungen zur Auswahl der verwendeten Strukturen
% overconstraint: Anzahl der Beingelenke, die die EE-FG übersteigen
% task redundancy: Anzahl der EE-FG, die die Aufgaben-FG übersteigen
structures = struct( ...
  'use_serial', true, ... % Wähle serielle Roboter
  'use_parallel', true, ... % Wähle parallele Roboter
  'use_parallel_rankdef', false, ... Nehme auch parallele Roboter, die mit Rangverlust in der Datenbank stehen
  'onlylegchain_from_synthesis', true, ... % Nehme keine seriellen Ketten als Beinkette, die nur manuell in die SerRobLib eingetragen wurden
  'use_kinematic_variants', true, ... % Nehme auch serielle Ketten, die eine Variante eines allgemeinen Modells sind
  'maxnumprismatic', 1, ... % Maximal ein Schubgelenk in Roboter (bzw. PKM-Beinkette)
  'max_task_redundancy', 0, ... % Zulässiger Grad der Aufgabenredundanz
  'max_kin_redundancy', 0, ... % Zulässiger Grad der kinematischen Redundanz
  'DoF', input_settings.DoF, ... % FG des Roboters in Notation [1 1 1 0 0 1] (Geschw., Winkelgeschw.)
  'joint_filter', '******', ... % Vorgabe von Gelenktypen ("R", "P", "*").
  'parrob_basejointfilter', 1:9, ... % Vorgabe zum Gestell-Koppelgelenktyp einer PKM
  'nopassiveprismatic', true, ... % Schubgelenke dürfen nicht passiv sein
  'activenotlastjoint', true, ... % Verhindert ein aktives Plattform-Koppelgelenk
  'max_index_active', 6, ... % Setzt den maximalen Index aktuierter Gelenke fest (nachrrangig gegen vorherige Option)
  'mounting_serial', {'floor'}, ... % Montageort für serielle Roboter: floor, ceiling, wall
  'mounting_parallel', {'ceiling'}, ... % ... für PKM (wird entsprechend zur Aufgabe ausgerichtet; Schubgelenke haben Vorzugsrichtung)
  'whitelist', {''}); % Liste, die die Systeme beschränkt

%% Optimierungs-Einstellungen
% Einstellungen mit Auswirkung auf die Optimierung: Auswahl der
% Optimierungsvariablen und Annahmen über die Roboter, die getroffen werden
optimization = struct( ...
  'objective', {'energy'}, ... % Zielfunktion. Möglich: mass, energy, condition, 
   ... % valid_kin, valid_act, actforce, materialstress, stiffness, jointrange, manipulability, 
   ... % minjacsingval, positionerror; auch mehrere gleichzeitig möglich.
  'obj_jointrange', ... % Zusatzeinstellungen für die Zielfunktion "jointrange"
    struct( 'only_revolute', true, ... % Minimiere nur Wertebereich von Drehgelenken
            'only_passive', true), ... % Minimiere nur Wertebereich passiver Gelenke
  'constraint_obj', zeros(6,1), ... % Nebenbedingungen, 1=Mass, 2=Energy, 3=Actforce, 4=Condition, 5=Stiffness, 6=MaterialStress; Eintrag entspricht physikalischem Wert
  'movebase', true, ... % Position der Roboter-Basis
  'basepos_limits', NaN(3,2), ... % Grenzen für Basis-Position (Absolut, im Welt-KS)
  'ee_translation', true, ... % Freie Verschiebung des EE
  'ee_translation_only_serial', true, ... % ... nur bei seriellen Robotern
  'ee_rotation', true, ... % Freie Rotation des EE
  'base_size', true, ... % Größe des Gestells
  'base_size_limits', [NaN, NaN], ... % Grenzen für Gestell-Größe (Radius; Absolut)
  'platform_size', true, ... % Größe der Plattform
  'platform_size_limits', [NaN, NaN], ... % Grenzen für Plattform-Größe (Radius; Absolut)
  'base_morphology', false, ... % Aussehen des Gestells (z.B. Schrägheit, Gelenkpaarabstand)
  'platform_morphology', false, ... % Aussehen der Plattform (z.B. Gelenkpaarabstand)
  'rotate_base', false, ... % Orientierung der Roboter-Basis
  ... % Begrenzung des Drehbereichs von Gelenken. Größere Drehung für Gelenk
  ... % selbst unproblematisch. Eher Kollision bei mehrfacher Drehung schwer zu kontrollieren
  'max_range_active_revolute', 400*pi/180, ... % Maximaler Drehwinkel aktiver Drehgelenke (mehr als volle Drehung für Motoren unproblematisch)
  'max_range_passive_revolute', 720*pi/180, ... % Maximaler Drehwinkel passiver Drehgelenke (mechanisch geht unendlich oft bei Drehgelenken, bei Kugel/Kardan deutlich weniger)
  'max_velocity_passive_revolute', 20, ... % [rad/s] Maximale Drehgeschw. (zur Singularitätsvermeidung)
  'max_velocity_active_revolute', 8, ... % [rad/s] Maximale Drehgeschw. (zur Singularitätsvermeidung)
  'max_velocity_active_prismatic', 5, ... % [m/s] Maximale Geschw. (zur Singularitätsvermeidung)
  'desopt_vars', {{}}, ... % Variablen für eigene Optimierung der Entwurfsparameter. Möglich: "linkstrength", "joint_stiffness_qref"
  'safety_link_yieldstrength', 1, ... % Sicherheitsfaktor für Streckgrenze der Segmente als Nebenbedingung. Berechnung gesteuert über constraint_obj(6)
  'constraint_collisions', false, ... Schalter für Kollisionsprüfung
  'obj_limit', 0, ... % Grenze des Fitness-Wertes zum Beenden der Simulation
  'obj_limit_physval', 0, ... % Grenze für den physikalischen Wert zum Beenden
  'NumIndividuals', 50, ... % Anzahl der Partikel im PSO
  'MaxIter', 10, ... % Anzahl der Iterationen im PSO (zusätzlich zur initialen)
  'static_force_only', false, ... % Betrachte nur statische Kraft, keine Dynamik (egal ob Geschwindigkeit/Beschleunigung gegeben)
  'joint_stiffness_passive_revolute', 0, ... % Zur Annahme von Drehfedern in den Gelenken. Ist Sonderfall für Festkörpergelenke.
  'nolinkmass', false, ... % Setze die Masse der Robotersegmente auf Null.
  'noplatformmass', false, ... % Setze die Masse der PKM-Plattform auf Null.
  'ElectricCoupling', true, ... % Kopplung der Achsen für Energieberechnung. TODO
  'InitPopRatioOldResults', 0.50, ... % 75% der Initialpopulation aus bisherigen Ergebnissen generieren
  'result_dirs_for_init_pop', {{}}, ... % Zusätzliche Verzeichnisse zum Laden der Initialpopulation
  'resdir', fullfile(fileparts(which('structgeomsynth_path_init.m')), 'results'), ...
  'optname', 'unnamed');

%% Einstellungen für Aufgabe (Trajektorie, Bauraum, Hindernisse)
task = struct( ...
  'profile', 1, ... % Beschleunigungs-Trapez für Kartesische Eckpunkte
  'vmax', 1, ... % maximale Geschwindigkeit (m/s oder rad/s)
  'amax', 3, ... % maximale Beschleunigung (m/s² oder rad/s²)
  'Tv', 0.01, ... % Verschliffzeit der Beschleunigung (für Ruckbegrenzung)
  'Ts', 1e-3, ... % Abtastzeit der Trajektorie
  'maxangle', 2*pi, ... % Keine Einschränkung für die maximalen Winkel
  'installspace', struct( ... % Konfiguration des möglichen Bauraums
    'links', {{}}, ... % jew. Liste der von der Basis gezählten Segmente (Bsp.: alle erlaubt ist 0:6)
    'type', [], ... % 1=Quader, 2=Zylinder; zeilenweise mehrere Körper
    'params', []), ... % im Welt-KS. Jeweils die geometrie-beschreibenden Parameter
  'obstacles', struct( ... % Hindernisse im Arbeitsraum zur Kollisionsprüfung
    'type', [], ... % Nummerierung siehe SerRob.m (collbodies)
    'params', []), ...% s.o.
  'payload', struct('m', 3, 'rS', zeros(3,1), 'Ic', zeros(6,1)));
task.payload.Ic(1:3) =  2/5 * task.payload.m * (60e-3)^2; % Kugel Radius 60mm
%% Rückgabe Gesamt-Einstellungen
settings = struct(...
  'structures', structures, ...
  'optimization', optimization, ...
  'task', task, ...
  'general', general);
