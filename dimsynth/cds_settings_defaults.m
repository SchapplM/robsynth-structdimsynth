% Standard-Einstellungen für kombinierte Struktur- und Maßsynthese
% 
% Eingabe:
% input_settings
%   Struktur mit Einstellungen zur Generierung aller
%   Optimierungseinstellungen. Felder:
%   DoF: FG der Aufgabe in Notation [1 1 1 0 0 1] (Geschw., Winkelgeschw.)
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
defstruct = cds_definitions();
%% Allgemeine Einstellugen
% Einstellungen mit Auswirkungen auf vom Benutzer sichtbare Ergebnisse und
% deren Nachverarbeitung
general = struct( ...
  'verbosity', 2, ... % Ausgabeleven;0=minimal (Endergebnis), 1=mehr (Generationenweise), 2=noch mehr (Jede Fitness-Eval.), 3=alles, 4=Auch Entwurfsoptimierung
  'matfile_verbosity', 0, ... % Menge an mat-Dateien, die für Debug-Zwecke gespeichert werden; 0=keine, 1=wenige, 2=viele (erfolgreiche Ind.) 3=alle
  'plot_robot_in_fitness', 0, ... % Schwellwert der Gütefunktion zum Zeichnen von Details.
  'plot_details_in_fitness', 0, ... % Positiv: nur bei besseren; negativ: nur bei schlechteren als ...
  'plot_details_in_desopt' , 0, ... % Wie vorheriges Argument, aber für die Gütefunktion der Entwurfsoptimierung
  'taskred_dynprog', false, ... % Benutze im Fall von Redundanz die dynamische Programmierung
  'taskred_dynprog_mode', 'discrete', ... % Redundanzauflösung in DP. Möglichkeiten: 'discrete', 'continuous'
  'taskred_dynprog_and_gradproj', true, ... % Benutze zusätzlich noch die normale IK-Funktion ohne DP
  'taskred_dynprog_only', false, ... % Keine Gradientenprojektion aufbauend auf DynProg-Trajektorie
  'taskred_dynprog_numstates', [6 12], ... % Anzahl der Zustände für die Dynamische Programmierung im ersten und zweiten Durchlauf. Ein Zustand prüft lediglich die konstante Orientierung.
  ... % Debug-Einstellungen um gezielt einige Bilder und Untersuchungen zu aktivieren
  'debug_taskred_perfmap', 0, ...% Redundanzkarte (Rasterung des redundanten FG über Trajektorie). 0=aus, 1=nur Summe, 2=Details (eine Karte für jedes Leistungsmerkmal getrennt)
  'debug_taskred_fig', false, ... % Diverse weitere Plots zur Aufgabenredundanz
  'debug_dynprog_files', false, ... % Speichert alle Zwischenzustände der Dynamischen Programmierung ab. Für Produktiv-Betrieb zu große Datenmenge.
  'debug_desopt', false, ... % Speichere das komplette Ergebnis der Entwurfsoptimierung ab für spätere Auswertungen
  'save_robot_details_plot_fitness_file_extensions', {''}, ... % Speichern des durch vorherige Einstellung erstellten Bildes
  'save_animation_file_extensions', {{'mp4'}}, ... % Format, in denen die Animationen gespeichert werden
  'animation_styles', {{'stick'}}, ... % Visualisierungsarten im Video: stick,3D,collision; bei mehr als einem: Syntax {{'1.','2.'}}
  'animation_installationspace', false, ... % Zeichne die Grenzen des Arbeitsraums in Animation mit ein
  'animation_workspaceobstacles', false, ... % Zeichne (Kollisions-)Störobjekte in die Animation mit ein
  'maxduration_animation', 10, ... % Die Animation soll max. 10s dauern (als Videodatei)
  'save_evolution_video', false, ... % Video mit Evolution der Roboter
  'evolution_video_frametime', 0.2, ... % Anzeigedauer eines Einzelbildes im Evolutionsvideo.
  'max_retry_bestfitness_reconstruction', 2, ... % Anzahl Neuversuche zur Reproduktion. 2 reichen zur Prüfung, ob Wiederholbarkeit da ist.
  'regenerate_summary_only', false, ... % Nur die Videos und Zusammenfassungsbilder neu generieren. Keine Optimierung durchführen.
  'only_finish_aborted', false, ... % Führe keine Optimierung durch, sondern werte abgebrochene vorherige Optimierungen aus
  'overwrite_existing_results', true, ... % Führe die Maßsynthese erneut durch, auch wenn schon Daten vorhanden sind. Für alleinige Berechnung unfertiger Roboter auf false setzen
  'nosummary', false, ... % Kompletter Verzicht auf die graphische Endauswertung
  'eval_figures', {{'histogram', 'fitness_various', 'jointtraj', ... % Liste der zu erstellenden Bilder. ...
    'robvisuanim', 'pareto_all_phys', 'pareto_all_fval', 'pareto'}}, ... % ... Auswahl, siehe cds_vis_results.m
  'only_save_summary_figures', true, ... % Bilderzeugung dient hauptsächlich der Speicherung. Schließe alle Bilder nach Speicherung
  'noprogressfigure', true, ... % Verzicht auf Fortschritts-Bild des PSO (meistens nicht hilfreich)
  'debug_calc', false, ... % Doppelte Berechnung zur Prüfung von Funktionen
  'parcomp_struct', 0, ... % Parallele Berechnung unterschiedlicher Roboter. Enthält Anzahl der Worker-Instanzen als Zahl
  'parcomp_plot', 0, ... % Parallele Erzeugung der Ergebnisbilder und -videos
  'parcomp_maxworkers', inf, ... % Beschränkung der Anzahl paralleler Instanzen
  ... % Einstellungen zur Benutzung des Clusters. Siehe auch https://www.luis.uni-hannover.de/de/services/computing/scientific-computing/
  'computing_cluster', false, ... % Berechnung auf PBS-Cluster (Hochladen auf Server)
  'computing_cluster_cores', 16, ... % Anzahl der benutzten Kerne  pro Node auf dem Cluster (16 ist vergleichsweise flexibel)
  'computing_cluster_max_time', NaN, ... % maximale Rechenzeit pro Roboter (in Sekunden). Bedingt voll-parallele Berechnung
  'computing_cluster_start_time', NaN, ... % Zeitpunkt, zu dem der Cluster-Job gestartet wurde. Zeitformat von `now`
  'continue_with_part', NaN, ... % Falls Starten von aufgeteilten Jobs abgebrochen wurde, kann bei einem bestimmten Part das Hochladen auf das Cluster fortgesetzt werden
  'isoncluster', false, ... % Merker, ob gerade auf dem Cluster gerechnet wird. Dann sind einige Bilder und Debug-Auswertungen unnötig.
  ... % Unbegrenzte Anzahl von Robotern auf jeder Cluster-Node. Niedrigere 
  ... % Zahl dient zur stärkeren Parallelisierung. Bei "inf" nur eine 
  ... % Cluster-Node. Bei NaN wird der Wert aus computing_cluster_cores 
  ... % benutzt. Das ist die sinnvollste Einstellung, wenn das Ergebnis 
  ... % schnellstmöglich fertig werden soll.
  'cluster_maxrobotspernode', NaN, ... 
  ... % Liste von Job-IDs, die zuerst fertig werden sollen (bzw. Abbruch o.ä.)
  'cluster_dependjobs', struct('afterok', [], 'afternotok', [], 'afterany', []), ...
  ... % Optionen zur Benutzung kompilierter Funktionen
  'compile_missing_functions', true, ... % Bei Start alle mex-Funktionen kompilieren
  'create_template_functions', false, ... % Erzeuge Funktionen neu aus Vorlagen-Dateien (immer)
  'update_template_functions', true, ... % Prüfe die Vorlagen-Dateien (Aktualisierung, falls veraltet)
  'check_missing_template_functions', true, ... % Prüfe auf fehlende Vorlagen-Dateien
  'use_mex', true);

%% Einstellungen zur Auswahl der verwendeten Strukturen
% PKM zeigen standardmäßig von der Decke herunter. Ausnahme: 2T1R. Dort
% nicht sinnvoll, da die Höhe nicht änderbar ist.
if all(input_settings.DoF==[1 1 0 0 0 1]) || all(input_settings.DoF==[1 1 0 0 0 0])
  mounting_parallel_default = 'floor';
else
  mounting_parallel_default = 'ceiling';
end

structures = struct( ...
  'use_serial', true, ... % Wähle serielle Roboter
  'use_parallel', true, ... % Wähle parallele Roboter
  'use_parallel_rankdef', false, ... Nehme auch parallele Roboter, die mit Rangverlust in der Datenbank stehen
  'onlylegchain_from_synthesis', true, ... % Nehme keine seriellen Ketten als Beinkette, die nur manuell in die SerRobLib eingetragen wurden
  'use_kinematic_variants', true, ... % Nehme auch serielle Ketten, die eine Variante eines allgemeinen Modells sind
  'only_serialrobot_from_synthesis', true, ... % Nehme nur allgemeine Robotermodelle aus Ramirez-Struktursynthese (keine Varianten)
  'maxnumprismatic', 1, ... % Maximal ein Schubgelenk in Roboter (bzw. PKM-Beinkette)
  'min_task_redundancy', 0, ... % Geforderter Grad der Aufgabenredundanz
  'max_task_redundancy', 0, ... % Zulässiger Grad der Aufgabenredundanz
  'max_kin_redundancy', 0, ... % Zulässiger Grad der kinematischen Redundanz
  'min_task_constraint', 0, ...% Minimale Anzahl kinematischer Beschränkungen des Endeffektors. 
  'max_task_constraint', 0, ... % Bei "3" können auch 3T3R-Roboter für 3T0R-Aufgaben benutzt werden.
  'joint_filter', {{'******'}}, ... % Vorgabe von Gelenktypen ("R", "P", "*"). Mehrere Filter in Cell-Array möglich (ODER-Verknüpfung)
  'num_tech_joints', 1:6, ... Mögliche Anzahl technischer Gelenke (in PKM-Beinketten). Wert 3 ermöglicht bspw. Ketten UPS, PUS, RUS, ...
  'parrob_basejointfilter', 1:10, ... % Vorgabe zum Gestell-Koppelgelenktyp einer PKM (alle Gelenke) ...
  'parrob_basejointfilter_prismatic', 1:10, ... % ... im Fall eines Schubgelenks am Gestell (überstimmt vorherige)
  'parrob_basejointfilter_revolute', 1:10, ... % ... im Fall eines Drehgelenks am Gestell
  'parrob_platformjointfilter', 1:9, ... % Vorgabe zum Gestell-Koppelgelenktyp einer PKM
  'prismatic_cylinder_no_lever', true, ... % Bei Hubzylindern (in PKM) darf kein zusätzlicher Heben wirken. Direkte Verbindung der Gelenke
  'orthogonal_links', false, ... % Bei true werden keine schräg abgehenden Segmente erlaubt (d-Parameter der DH-Notation)
  'orthogonal_joints', false, ... % Bei true werden keine schräg angebrachten Gelenke benutzt (alpha-Parameter der DH-Notation)
  'nopassiveprismatic', true, ... % Schubgelenke dürfen nicht passiv sein
  'activenotlastjoint', true, ... % Verhindert ein aktives Plattform-Koppelgelenk
  'max_index_active', 6, ... % Setzt den maximalen Index aktuierter Gelenke fest (nachrrangig gegen vorherige Option); für PKM
  'max_index_active_revolute', 6, ... % wie vorherige, nur bezogen auf Drehgelenke; für PKM
  'max_index_active_prismatic', 6, ... % wie vorherige, nur bezogen auf Schuzbgelenke; für PKM
  'no_inactive_joints', true, ... % Kinematiken mit inaktiven Gelenken werden verworfen
  'mounting_serial', {'floor'}, ... % Montageort für serielle Roboter: floor, ceiling, wall
  'mounting_parallel', {mounting_parallel_default}, ... % ... für PKM (wird entsprechend zur Aufgabe ausgerichtet; Schubgelenke haben Vorzugsrichtung)
  'repeatlist', {{}}, ... % Liste für mehrfache Durchführung eines Roboters. Einträge: {'Name', Anzahl}. Sinnvoll, wenn parallele Berechnung möglich.
  'whitelist', {''}); % Liste, die die Systeme beschränkt

%% Optimierungs-Einstellungen
% Einstellungen mit Auswirkung auf die Optimierung: Auswahl der
% Optimierungsvariablen und Annahmen über die Roboter, die getroffen werden
optimization = struct( ...
  'objective', {{'TODO'}}, ... % Zielfunktion. Möglich: mass, energy, power, condition, 
   ... % valid_kin, valid_act, actforce, materialstress, stiffness, jointrange, jointlimit
   ... % manipulability, minjacsingval, positionerror, actvelo, chainlength,
   ... % installspace, footprint, colldist, mrk1, mrk2, mrk3, mrk4. Auch mehrere gleichzeitig möglich.
  'obj_jointrange', struct(... % Zusatzeinstellungen für die Zielfunktion "jointrange"
      'only_revolute', false, ... % Minimiere nur Wertebereich von Drehgelenken
      'only_prismatic', false, ... % Minimiere nur Wertebereich von Schubgelenken
      'only_active', false, ... % Minimiere nur Wertebereich aktiver Gelenke
      'only_passive', false), ... % Minimiere nur Wertebereich passiver Gelenke
  'obj_power', ... % Zusatzeinstellungen für die Zielfunktion "power"
  ... Nehme nicht das Maximum der Einzel-Leistungen, sondern die Leistung 
  ... aus max. Drehmoment und Drehzahl. Betrifft PKM:
    struct( 'symmetric_speed_torque_limits', true), ...
  ... Konfigurierbarkeit für Genauigkeit der Antriebe
  'obj_positionerror', struct(... % Zusatzeinstellungen für die Zielfunktion "jointrange"
      'revolute', 7 * 1/3600 * pi/180, ... % Genauigkeit: 7 Winkelsekunden; Umrechnung in Grad und Radiant; https://www.heidenhain.de/de_DE/produkte/winkelmessgeraete/winkelmessmodule/baureihe-mrp-2000/
      'prismatic', 10e-6), ... % % https://www.heidenhain.de/de_DE/produkte/laengenmessgeraete/gekapselte-laengenmessgeraete/fuer-universelle-applikationen/
  ... Zielgröße für IK bei Redundanz. Möglich:
  ...  * default (Einstellung anhand der Kriterien der Maßsynthese), 
  ...  * ikjac_cond, jac_cond, coll_par, instspc_par, poserr_ee (siehe invkin-Funktionen)
  ...  * none (keine Optimierung, nur geringste Beschleunigung), 
  ...  * constant (benutzt eine konstante Orientierung für alle Eckpunkte und für die ganz Trajektorie. Entspricht Fall ohne Redundanz) 
  'objective_ik', {{'default'}}, ... 
  'criteria_config_selection', {{}}, ... % Zielkriterien (mehrere möglich) zur Auswahl einer Konfiguration, falls mehrere gleich gut sind
  'constraint_obj', zeros(length(defstruct.objconstr_names_all),1), ... % Nebenbedingungen, 1=Mass, 2=Energy, 3=Actforce, 4=Condition, 5=Stiffness, 6=MaterialStress, 7=PositionError, 8=MRK1, 9=MRK2, 10=MRK3; Eintrag entspricht physikalischem Wert
  'condition_limit_sing', 1e5, ... % Wenn die Konditionszahl (der IK-Jacobi) schlechter ist, wird sofort abgebrochen. Schwellwert für Singularität. Deaktivieren durch setzen auf inf.
  'condition_limit_sing_act', inf, ... % Wenn die Konditionszahl (der PKM-Jacobi) schlechter ist, wird sofort abgebrochen. Schwellwert für Singularität. Deaktivieren durch setzen auf inf.
  'algorithm', 'pso', ... % Optimierungsalgorithmus. Möglich: Einkriteriell: pso; Mehrkriterielle: mopso, gamultiobj
  'movebase', true, ... % Position der Roboter-Basis
  'basepos_limits', NaN(3,2), ... % Grenzen für Basis-Position (Absolut, im Welt-KS)
  'ee_translation', true, ... % Freie Verschiebung des EE
  'ee_translation_fixed', NaN(1,3), ... % vorgegebene EE-Transformation (bspw. bereits konstruierter Endeffektor). Entspricht r_N_E (SerRob) bzw. r_P_E (ParRob)
  'ee_translation_only_serial', true, ... % ... nur bei seriellen Robotern
  'ee_rotation', true, ... % Freie Rotation des EE
  'ee_rotation_fixed', NaN(1,3), ... % vorgegebene EE-Transformation (bspw. bereits konstruierter Endeffektor). Entspricht phi_N_E (SerRob) bzw. phi_P_E (ParRob); XYZ-Euler-Winkel
  'max_chain_length', inf, ... % maximale Länge der Beinketten (bei PKM) bzw. max. Länge der seriellen Kinematik (in m)
  'base_size', true, ... % Größe des Gestells
  'base_size_limits', [NaN, NaN], ... % Grenzen für Gestell-Größe (Radius; Absolut; bei paarweiser Anordnung wird effektiver Radius gezählt)
  'base_tolerance_prismatic_guidance', 1.0, ... % Erhöhte Toleranz für das Überstehen von Schubgelenk-Führungsschienen
  'platform_size', true, ... % Größe der Plattform
  'platform_size_limits', [NaN, NaN], ... % Grenzen für Plattform-Größe (Radius; Absolut; bei paarweiser Anordnung wird effektiver Radius gezählt)
  'max_platform_base_ratio', inf, ... % Plattform sollte nicht so viel mal größer als die Basis sein
  'base_morphology', true, ... % Aussehen des Gestells (z.B. Schrägheit, Gelenkpaarabstand)
  'platform_morphology', true, ... % Aussehen der Plattform (z.B. Gelenkpaarabstand)
  'tilt_base', false, ... % Kippen der Roboter-Basis (über Rotation um x- und y-Achse). Ermöglicht Ausweichen struktureller Singularitäten
  'max_tilt_base', pi, ... % Begrenzung des Kippwinkels der Basis (sowohl einzelne Euler-Winkel, als auch Gesamt-Neigung)
  'tilt_base_only_orthogonal', false, ... % Nur glatte 90°-Winkel
  'rotate_base', true, ... % Orientierung der Roboter-Basis (nur um die z-Achse). Hilft bei PKM.
  'rotate_base_only_orthogonal', false, ... % Nur glatte 90°-Winkel zulassen
  ... % Fixiere die Gelenkwinkel-Grenzen auf den beim Start gesetzten Wert.
  ... % Ist nur sinnvoll, wenn ein bereits gegebenes Robotermodell genutzt wird.
  'fix_joint_limits', false, ...
  ... % Begrenzung des Verfahrweges von Schubgelenken. Mit NaN deaktiviert 
  ... % (wird dann aus Größe des Roboters plausibel abgeleitet).
  ... % Hier nur "Bereich", also "Hub". Konstanter Längenversatz zusätzlich
  'max_range_prismatic', NaN, ...
  ... % Begrenzung des Drehbereichs von Gelenken. Größere Drehung für Gelenk
  ... % selbst unproblematisch. Eher Kollision bei mehrfacher Drehung schwer zu kontrollieren
  ... % Maximaler Drehwinkel aktiver Drehgelenke (mehr als volle Drehung 
  ... % für Motoren unproblematisch). Angabe ist immer Spannweite
  'max_range_active_revolute', 400*pi/180, ...
  ... % Maximaler Drehwinkel passiver Drehgelenke (mechanisch geht unend-
  ... % lich oft bei Drehgelenken; bei einigen PKM drehen die Gelenke so oft, 
  ... % wie bspw ein Objekt mit dem EE "umkreist" wird. 
  'max_range_passive_revolute', inf*pi/180, ... 
  ... % % Drehwinkel bei Kardan- und Kugelgelenken technisch deutlich 
  ... % niedriger. Hier vorerst nur grobe Annahmen.
  'max_range_passive_universal', pi, ... % 90° in jede Richtung
  'max_range_passive_spherical', 120*pi/180, ... % 60° in jede Richtung
  'max_velocity_passive_revolute', 20, ... % [rad/s] Maximale Drehgeschw. (zur Singularitätsvermeidung)
  'max_velocity_passive_universal', 20, ...
  'max_velocity_passive_spherical', 20, ...
  'max_velocity_active_revolute', 8, ... % [rad/s] Maximale Drehgeschw. (zur Singularitätsvermeidung)
  'max_velocity_active_prismatic', 5, ... % [m/s] Maximale Geschw. (zur Singularitätsvermeidung)
  'max_acceleration_prismatic', 50, ... % ca. 5g Beschleunigung ist technisch kaum zu realisieren
  'max_acceleration_revolute', 100, ... % Maximale Geschw. (20) wäre in 0.2s erreicht. Sehr hoher Wert für frühe Erkennung schlechter Konditionierung
  'max_velocity_ee_rotation', 2*pi, ... % [rad/s]; im Fall von Aufgabenredundanz maximale Drehgeschwindigkeit des Endeffektors
  'max_acceleration_ee_rotation', 2*pi/0.200, ... % rad/s²; Aufbau der max. Geschwindigkeit in 200ms (sehr dynamisch)
  'ee_rotation_limit', [-inf, inf], ... % Grenzen für die EE-Drehung bei Vorliegen von Aufgabenredundanz
  'ee_rotation_limit_rel_traj', [-inf, inf], ... % relative Grenzen für EE-Drehung in Trajektorie (bzgl. Start-Drehung aus Positions-IK)
  'joint_limits_symmetric_prismatic', true, ... % Wähle die Gelenkgrenzen bei Schubgelenken symmetrisch (für Führungsschienen)
  'joint_limits_symmetric_revolute', false, ... % Auch für Drehgelenke wählen. Sinnvoll, wenn Auslegung für Dreh-/Kardan-/Kugelgelenk genau symmetrisch sein soll.
  'check_jointrange_points', true, ... % Prüfung der Gelenkwinkelspannweite bereits bei den Eckpunkten (Möglichkeit für falsch-positive Ausschlüsse)
  'prismatic_cylinder_allow_overlength', false, ... % Bei Schubzylindern darf der Zylinder nicht durch das vorherige Gelenk gehen. Ist konstruktiv ungünstig.
  'desopt_vars', {{}}, ... % Variablen für eigene Optimierung der Entwurfsparameter. Möglich: "linkstrength", "joint_stiffness_qref", "joint_stiffness"
  'safety_link_yieldstrength', 1, ... % Sicherheitsfaktor für Streckgrenze der Segmente als Nebenbedingung. Berechnung gesteuert über constraint_obj(6)
  'constraint_collisions', false, ... Kollisionsprüfung
  'constraint_collisions_desopt', false, ... % Kollisionsprüfung in Entwurfsoptimierung (mit Änderung der Segmentdurchmesser)
  'collision_bodies_size', 40e-3, ... % Durchmesser der Kollisionskörper (meistens Kapseln)
  'collision_bodies_safety_distance', 10e-3, ... % Zusätzlicher Sicherheitsabstand um die Segmente (entspricht zusätzlichem Radius, für constraint_collisions_desopt)
  'single_point_constraint_check', false, ... % Prüfe einige Nebenbedingungen für jeden Arbeitsraum-Eckpunkt. Schnellere Prüfungen, aber voraussichtlich schlechtere Konvergenz
  'symmetric_assembly_mode', false, ... % Erzwinge mit true einen symmetrischen Einbau aller Beinketten
  'collshape_base', {{'default'}}, ... % Form der Kollisionskörper für die PKM-Basis: default, star, ring, joint. Siehe cds_update_collbodies.m; mehrere möglich.
  'collshape_platform', {{'default'}}, ... % Form der Kollisionskörper für die PKM-Plattform default, star, ring, sphere. Siehe cds_update_collbodies.m; mehrere möglich.
  ... % Anzahl der Versuche für die Positions-IK erhöhen (für Reproduktion 
  ... % der Ergebnisse bei anderer Zufallszahlen-Grundlage). Dient auch als 
  ... % Schalter für IK-Einstellungen mit Fokus auf Reproduzierbarkeit. 
  ... % Kann auch negativ gewählt werden, um die Zahl der Versuche zu 
  ... % reduzieren (damit es schneller geht bei nachträglicher Auswertung)
  'pos_ik_tryhard_num', 0, ... 
  'pos_ik_abort_on_success', false, ... % Bei true wird nach einer gefundenen IK-Konfiguration abgebrochen (in cds_constraints)
  'traj_ik_abort_on_success', false, ... % ... (in cds_constraints_traj)
  'obj_limit', 0, ... % Grenze des Fitness-Wertes zum Beenden der Simulation
  'obj_limit_physval', NaN, ... % Grenze für den physikalischen Wert zum Beenden
  'NumIndividuals', 50, ... % Anzahl der Partikel im PSO
  'MaxIter', 10, ... % Anzahl der Iterationen im PSO (zusätzlich zur initialen)
  'max_time', inf, ... % Zeit für die Optimierung eines Roboters, in Sekunden
  'start_time', NaN, ... % Start-Zeit für Optimierung eines konkreten Roboters. Im Format von Funktion now()
  'abort_pareto_front_size', inf, ... % Mit dem Wert kann die Optimierung vorzeitig bei ausreichend gefüllter Pareto-Front abgebrochen werden
  'desopt_NumIndividuals', NaN, ... % Anzahl der PSO-Partikel bei der Entwurfsoptimierung. NaN ist Standard-Wert (abhängig von Anzahl der Variablen)
  'desopt_MaxIter', NaN, ... % Anzahl der PSO-Generationen bei der Entwurfsoptimierung.
  'desopt_use_obj_limit', true, ... % Nutze Option obj_limit auch in cds_dimsynth_desopt_fitness.m. Zum Debuggen deaktivieren.
  'static_force_only', false, ... % Betrachte nur statische Kraft, keine Dynamik (egal ob Geschwindigkeit/Beschleunigung gegeben)
  'joint_stiffness_passive_revolute', 0, ... % Zur Annahme von Drehfedern in den Gelenken. Ist Sonderfall für Festkörpergelenke. NaN, falls Steifigkeit optimiert wird.
  'joint_stiffness_active_revolute', 0, ... % Das gleiche für aktive Drehgelenke
  'joint_stiffness_passive_universal', 0, ... % Gleiche Annahme für Kardangelenke (Sonderfall für MHI-Kryo-PKM)
  'joint_stiffness_min', 0, ... % Falls die Gelenksteifigkeit optimiert wird, ...
  'joint_stiffness_max', 100, ... % ... ist dies der maximale Wert (in Nm/rad). 100Nm/rad) sind 1.7Nm/Grad
  'min_inclination_conic_base_joint', 5*pi/180, ... % Die Neigung konischer Gestellgelenke soll nicht waagerecht oder senkrecht sein können
  'max_inclination_conic_base_joint', pi, ... % Maximaler Winkel des Gestellgelenks gegen die Horizontale
  'min_inclination_conic_platform_joint', 5*pi/180, ... % Das gleiche für Plattformgelenke (nur bei konischer Anordnung)
  'min_joint_distance', 0, ... % Minimaler Abstand zwischen zwei Gelenken
  'nolinkmass', false, ... % Setze die Masse der Robotersegmente auf Null.
  'noplatformmass', false, ... % Setze die Masse der PKM-Plattform auf Null.
  'ElectricCoupling', true, ... % Kopplung der Achsen für Energieberechnung. TODO
  'InitPopRatioOldResults', 0.50, ... % Diesen Anteil der Initialpopulation aus bisherigen Ergebnissen generieren
  'InitPopFromGlobalIndex', false, ... % Benutze Index-Datei (index.mat) im Ergebnis-Order zum Laden alter Ergebnisse (deutlich schneller auf Cluster). Muss mit cds_gen_init_pop_index erstellt werden.
  'InitPopFromDetailResults', false, ... % Lade auch die Detail-Ergebnisse und benutze auch die nicht Pareto-optimalen Ergebnisse als mögliche Startwerte
  'result_dirs_for_init_pop', {{}}, ... % Zusätzliche Verzeichnisse zum Laden der Initialpopulation
  'resdir', fullfile(fileparts(which('structgeomsynth_path_init.m')), 'results'), ...
  'optname', 'unnamed');

%% Einstellungen für Aufgabe (Trajektorie, Bauraum, Hindernisse)
task = struct( ...
  'DoF', input_settings.DoF, ... % Für die Aufgabe relevante Freiheitsgrade
  'pointing_task', false, ... % Bei true ist die Drehung um die EE-z-Achse egal (für 3T0R/2T0R notwendig)
  .... % Wahl der Trajektorienart:
  ... % 0=nur Eckpunkte, 
  ... % 1=Beschleunigungs-Trapez (Rast-zu-Rast für kartesische Eckpunkte, Konsistente Beschleunigung), 
  ... % 2=Trajektorie ohne Rastpunkte (allgemeinere Trajektorie, oder aus Differenzenquotient, benutze cumsum statt cumtrapz, Konsistenz der Beschleunigung nicht so wichtig).
  'profile', 1, ... 
  'vmax', 1, ... % maximale Geschwindigkeit (m/s oder rad/s)
  'amax', 3, ... % maximale Beschleunigung (m/s² oder rad/s²)
  'Tv', 0.01, ... % Verschliffzeit der Beschleunigung (für Ruckbegrenzung)
  'T_dec_ns', 0, ... % Zeit zum Abbremsen der Nullraumbewegung zwischen Rastpunkten. Bei 0 kein Abbremsen sondern durchgängige Bewegung. Verhältnis vmax/amax wird bei NaN automatisch gebildet
  'Ts', 1e-3, ... % Abtastzeit der Trajektorie
  'maxangle', 2*pi, ... % Keine Einschränkung für die maximalen Winkel
  'wall_rotate_traj', false, ... % Schalter für Wandmontage, damit Trajektorie gedreht wird. Nur notwendig, falls Standard-Trajektorie benutzt werden soll.
  'installspace', struct( ... % Konfiguration des möglichen Bauraums
    'links', {{}}, ... % jew. Liste der von der Basis gezählten Segmente (Bsp.: alle erlaubt ist 0:6)
    'type', [], ... % 1=Quader, 2=Zylinder; zeilenweise mehrere Körper
    'params', []), ... % im Welt-KS. Jeweils die geometrie-beschreibenden Parameter
  'obstacles', struct( ... % Hindernisse im Arbeitsraum zur Kollisionsprüfung
    'type', [], ... % Nummerierung siehe SerRob.m (collbodies)
    'params', []), ...% s.o.
  'interactionspace', struct( ... % Konfiguration des möglichen MRK-Interaktionsraums
    'type', [], ... % 1=Quader, 2=Zylinder; zeilenweise mehrere Körper
    'params', [], ... % s.o.
    'check_clamp_angle', true, ... % Prüfung der Winkel in Gelenken
    'check_clamp_dist', true, ... % Prüfung der Abstände verschiedener Beinketten
    'ref_force', 140), ... % Referenzkraft, die erkannt werden muss
  'payload', struct( ... % Zusätzliche Masse
    'm', 3, ... % Masse
    'rS', zeros(3,1), ... % Schwerpunkt, bezogen auf EE-KS (nicht: Plattform-KS)
    'Ic', zeros(6,1), ... % Trägheitstensor bzgl. Schwerpunkt. Reihenfolge xx, yy, zz, xy, xz, yz.
    'If', NaN(6,1))); % Alternative Darstellung des Trägheitstensors, bezogen auf das EE-KS statt auf den Schwerpunkt.
task.payload.Ic(1:3) =  2/5 * task.payload.m * (60e-3)^2; % Kugel Radius 60mm
%% Rückgabe Gesamt-Einstellungen
settings = struct(...
  'structures', structures, ...
  'optimization', optimization, ...
  'task', task, ...
  'general', general);
