# Toolbox für kombinierte Struktur- und Maßsynthese für serielle und parallele Roboter

Dieses Repo enthält die kombinierte Struktur- und Maßsynthese für serielle und parallele Roboter.

Mit dieser Software werden alle parallelen Roboterstrukturen für eine gegebene Aufgabe gefunden ("Struktursynthese") und diese Strukturen anschließend optimiert ("Maßsynthese"). Die Struktursynthese serieller Roboter erfolgt teilweise vorher mit der Struktursynthese von Daniel Ramirez.

Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-04  
(C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

## Benutzung

Für die Benutzung des Repos muss mit dem Skript `structgeomsynth_path_init.m` aus dem Hauptordner die Pfadinitialisierung durchgeführt werden.

### Abhängigkeiten

Die folgenden Toolboxen müssen heruntergeladen und deren Pfad in Matlab initialisiert sein:

* Robotik-Toolbox: [Github](https://github.com/SchapplM/robotics-toolbox), [Uni Hannover intern](https://gitlab.projekt.uni-hannover.de/imes-gruppe-robotik/matlab-robotik-toolbox) 
* Trajektorien-Toolbox: [Github](https://github.com/SchapplM/robotics-dep-traj), [Uni Hannover intern](https://gitlab.projekt.uni-hannover.de/imes-material/trajektorien-toolbox)
* Matlab-Toolbox: [Github](https://github.com/SchapplM/matlab_toolbox), [Uni Hannover intern](https://gitlab.projekt.uni-hannover.de/imes-material/matlab-toolbox)
* Externe Abhängigkeiten: [Github](https://github.com/SchapplM/robotics-dep-ext), [Uni Hannover intern](https://gitlab.projekt.uni-hannover.de/imes-material/matlab-ext)
* Geometrie-Toolbox: [Github](https://github.com/SchapplM/robotics-dep-geometry), [Uni Hannover intern](https://gitlab.projekt.uni-hannover.de/imes-material/geometrie-toolbox)
* Datenbank serieller Roboter: [Github](https://github.com/SchapplM/robsynth-serroblib), [Uni Hannover intern](https://gitlab.projekt.uni-hannover.de/imes-gruppe-robotik/serrob_mdlbib)
* Datenbank paralleler Roboter:  [Github](https://github.com/SchapplM/robsynth-parroblib), [Uni Hannover intern](https://gitlab.projekt.uni-hannover.de/imes-gruppe-robotik/parrob_mdlbib)
* PBS-Cluster-Transfer: [Github](https://github.com/SchapplM/matlab_pbs_transfer), [Uni Hannover intern](https://gitlab.projekt.uni-hannover.de/imes-material/matlab-luis-cluster)
  * Wird nur benötigt, falls die Maßsynthese parallel auf einem [PBS-/SLURM-Cluster](https://www.luis.uni-hannover.de/de/services/computing/scientific-computing/) gerechnet werden soll
  * Die Datei `dimsynth/dimsynth_cluster_header.m.template` muss angepasst und am gleichen Ort ohne template-Endung gespeichert werden. Für Struktursynthese auf dem Cluster muss zusätzlich `struktsynth_par/structsynth_cluster_header.m.template` angepasst werden.

### Tests

Die Maßsynthese wird durch verschiedene Beispiel-Aufrufe getestet. Die Test-Skripte liegen im Ordner `dimsynth/test`. Beispiele für die Maßsynthese liegen in `dimsynth/config`

* `structgeomsynth_test_repo.m`: Alle Tests des Repos ausführen (für Test-Framework)
* `cds_testcase_2T1R.m`: Maßsynthese für je zwei serielle und parallele Roboter mit verschiedenen Zielfunktionen.


### Struktursynthese

Der Hauptteil der Struktursynthese serieller Ketten erfolgt aktuell im anderen Synthese-Repo mit dem Code von Daniel Ramirez. Hier werden ausgewählte Eigenschaften der seriellen Ketten geprüft. Dazu dienen die Skripte in `struktsynth_ser`:

* `check_first_joint_orientation`: Prüfen, ob die Ausrichtung des ersten Gelenks dem vorgesehenen Schema entspricht.
* `serrob_change_last_joint_to_spherical`: Verändere bestehende allgemeine Strukturen in der Datenbank so, dass Beinketten mit Kugelgelenk am Ende als Varianten zusätzlich entstehen
* `serrob_isomorph_list`: Untersuche die Datenbank auf Isomorphismen
* `serrob_list_parameter_no_influence`: Prüfe, ob Parameter von Robotermodellen eventuell keinen Einfluss auf die Endeffektorbewegung haben.

Für die Struktursynthese paralleler Roboter existieren Skripte im Ordner `struktsynth_par`:

* `parroblib_add_robots_symact_alldof`: Alle symmetrisch aktuierten parallelen Roboter generieren und zur Datenbank ParRobLib hinzufügen
* `structsynth_2T1R.m`: Beispielskript für Synthese der PKM mit den FG 2T1R
  * genauso für 3T0R, 3T1R, 3T3R
* `structsynth_3T0R_cluster.m`: Beispielskript zur Durchführung der 3T0R-Struktursynthese auf dem Rechen-Cluster

### Maßsynthese

Die Maßsynthese erfolgt mit den Funktionen aus `dimsynth`. Der komplette Ablauf wird mit den Konfigurationsskripten aus `config` gestartet. Die Skripte bieten entweder Minimalbeispiele für bestimmte Freiheitsgrade oder führen die Maßsynthese für eine größere Menge an Robotern aus.
Der grobe Ablauf der Maßsynthese erfolgt durch die folgenden Matlab-Skripte und -Funktionen:

 * Start-Skript, z.B. `config/cds_example_2T1R.m` mit allen Einstellungen der Optimierung und Auswahl der Roboter (-Freiheitsgrade)
 * `cds_start`: Starten des Gesamt-Ablaufs
 * `cds_gen_robot_list`: Liste aller Roboter erzeugen (aus Datenbanken mit Filterkriterien)
 * `cds_dimsynth_robot`: Optimierung starten
 * `cds_gen_init_pop`: Erste Population generieren
 * `cds_fitness`: Diese Fitness-Funktion wird durch wiederholtes Aufrufen optimiert. Dort sind folgende Punkte enthalten:
   * `cds_update_robot_parameters`: Parameter aktualisieren
   * `cds_constraints`: Nebenbedingungen prüfen (bezüglich von einzelnen Punkten im Arbeitsraum)
   * `cds_constraints_traj`: ... bezüglich einer Trajektorie
   * `cds_constr_collisions_self`: Selbstkollisionen prüfen
   * `cds_constr_collisions_ws`: Kollision mit Arbeitsraum-Hindernissen prüfen
   * `cds_constr_installspace`: Einhaltung der Bauraumgrenzen prüfen
   * `cds_dimsynth_design`: Entwurfsparameter eintragen
   * `cds_dimsynth_desopt`: Entwurfsoptimierung durchführen
   * `cds_obj_dependencies`: Dynamik berechnen (als Abhängigkeit einiger Zielfunktionen)
   * `cds_obj_...`: Zielfunktion berechnen (z.B. `..._condition`, `..._energy`, `..._mass`, ...)
   * `cds_save_particle_details`: Speichert zusätzliche Informationen zu allen PSO-Partikeln
   * `cds_load_particle_details`: Lädt die Informationen nach der Optimierung
 * `cds_results_table`: Nach der Optimierung werden die Endergebnisse (optimale Parameter) jedes Roboters als Tabelle gespeichert
 * `cds_vis_results`: Nach der Optimierung erfolgt eine Darstellung der Ergebnisse. Die Bilder werden in einem Unterordner von `dimsynth/results` gespeichert
 * `cds_create_evolution_videos`: Ein Video aller Zwischenschritte der "Evolution" innerhalb des PSO wird erstellt
 
#### Entwurfsoptimierung

Optional kann in der Maßsynthese als innere Kaskade eine Optimierung der Entwurfsparameter durchgeführt werden.
Entwurfsparameter sind dabei Parameter, die nicht die Kinematik des Roboters beeinflussen. Also z.B. der Durchmesser und die Stärke der Segmente (im Gegensatz zu ihrer Länge).
Der Ablauf erfolgt in diesen Funktionen:

   * `cds_dimsynth_desopt`: Einrichtung und Start der Optimierung in der innersten Kaskaden
   * `cds_dimsynth_desopt_fitness`: Fitness-Funktion, die durch wiederholten Aufruf optimiert wird
   * `cds_obj_dependencies_regmult`: Aktualisierung der Dynamik-Parameter in der Entwurfsoptimierung
   * `cds_dimsynth_design`: Aktualisieren des Robotermodells basierend auf den optimierten Entwurfsparametern

# Quellen

Die kombinierte Struktur- und Maßsynthese wird in folgenden Quellen beschrieben:

* [SchapplerJahRaaOrt2021] Schappler, M., Jahn, P., Raatz, A., Ortmaier, T.: Combined Structural and Dimensional Synthesis of a Parallel Robot for Cryogenic Handling Tasks MHI-Tagung (2021)
* [SchapplerOrt2020] Schappler, M., Ortmaier, T.: Dimensional Synthesis of Parallel Robots: Unified Kinematics and Dynamics using Full Kinematic Constraints, IFToMM DACH Konferenz (2020)
* [SchapplerTapOrt2019] Schappler, M. and Tappe, S., Ortmaier, T.: Exploiting Dynamics Parameter Linearity for Design Optimization in Combined Structural and Dimensional Robot Synthesis, IFToMM World Congress (2019)
* [Ramirez2018] Ramirez, D.: Automatic Generation of Task-specific Mechanisms using Combined Structural and Dimensional Synthesis, Dissertation, Uni Hannover (2018)
